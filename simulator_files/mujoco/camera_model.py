"""HM01B0 camera model for the AI-deck frames of CrazySim.

crazysim_cpx.py runs every rendered frame through CameraModel before it is
streamed. The stages follow the light path:

  optics   lens distortion (Brown model), defocus blur, motion blur and
           rolling shutter (from the body rates), vignetting
  sensor   auto exposure (exposure time first, then analog gain), shot noise,
           read noise, pixel gain (PRNU) and offset (DSNU) maps, column offsets
  output   gamma and 8-bit quantization

A stage is off when its parameter is None (null in the profile). The fixed
pattern maps are drawn once per drone from the seed. Only numpy is needed.
"""
import math

import numpy as np

DEFAULTS = {
    'horizontal_fov': 1.50,     # rad, of the rendered image
    'stream_size': None,        # [w, h] of the streamed image; None: same as the render. A larger
                                #   render (render_scale) is resampled down through the lens model
    'gamma': 2.2,               # display gamma of the rendered image, undone first
    'output_gamma': 2.2,        # 1.0: linear raw output
    'distortion': None,         # {k1, k2, k3, p1, p2, fx, fy, cx, cy} of an OpenCV calibration
    'body_to_camera': None,     # 3x3 rotation body -> optical frame (x right, y down, z forward) for
                                #   motion blur and rolling shutter; None: looking along body +x (AI-deck)
    'defocus_sigma_px': None,
    'exposure_time_s': None,    # exposure at relative exposure 1 (motion blur)
    'row_time_s': None,         # rolling shutter readout time per row
    'vignetting': None,         # [v1, v2]: gain 1 - v1 r^2 - v2 r^4, r = 1 in the corner
    'vignetting_center': None,  # [cx, cy] in pixels (the principal point); None: image centre
    'auto_exposure': None,      # {target, tau_frames, max_exposure, max_gain}
    'full_well_e': None,        # electrons at full scale (shot noise)
    'read_noise_e': None,       # electrons rms (needs full_well_e)
    'prnu_std': None,           # pixel gain spread
    'dsnu_std_dn': None,        # pixel offset spread [DN]
    'column_noise_dn': None,    # column offset spread [DN]
    'black_level_dn': None,     # output offset [DN]; noise around it is clipped at 0 like on the sensor
}


def _shift(img, dx, dy):
    """Shift the image content by (dx, dy) pixels, bilinear, edges replicated."""
    h, w = img.shape
    if dx:
        xs = np.clip(np.arange(w) - dx, 0, w - 1)
        x0 = np.floor(xs).astype(np.intp)
        x1 = np.minimum(x0 + 1, w - 1)
        fx = (xs - x0).astype(img.dtype)
        img = img[:, x0] * (1 - fx) + img[:, x1] * fx
    if dy:
        ys = np.clip(np.arange(h) - dy, 0, h - 1)
        y0 = np.floor(ys).astype(np.intp)
        y1 = np.minimum(y0 + 1, h - 1)
        fy = (ys - y0).astype(img.dtype)[:, None]
        img = img[y0, :] * (1 - fy) + img[y1, :] * fy
    return img


def _gaussian_blur(img, sigma):
    radius = max(1, int(math.ceil(3 * sigma)))
    k = np.exp(-0.5 * (np.arange(-radius, radius + 1) / sigma) ** 2)
    k = (k / k.sum()).astype(img.dtype)
    padded = np.pad(img, radius, mode='edge')
    h, w = img.shape
    rows = sum(k[i] * padded[:, i:i + w] for i in range(2 * radius + 1))
    return sum(k[i] * rows[i:i + h, :] for i in range(2 * radius + 1))


def _binomial(img):
    """Separable [1 2 1]/4 low-pass (sigma 0.7 px), edges replicated."""
    p = np.pad(img, 1, mode='edge')
    rows = (p[:, :-2] + 2.0 * p[:, 1:-1] + p[:, 2:]) * 0.25
    return (rows[:-2] + 2.0 * rows[1:-1] + rows[2:]) * 0.25


def _motion_blur(img, length_x, length_y):
    """Average along the image motion during the exposure."""
    n = int(min(25, math.ceil(max(abs(length_x), abs(length_y))) + 1))
    if n < 2:
        return img
    offsets = np.linspace(-0.5, 0.5, n)
    return sum(_shift(img, o * length_x, o * length_y) for o in offsets) / n


def _rolling_shutter(img, shift_x_per_row, shift_y_per_row):
    """Rows are read out one after another: row r sees the scene moved by r times the shift."""
    h, w = img.shape
    rows = np.arange(h) - (h - 1) / 2.0
    if shift_y_per_row:
        ys = np.clip(np.arange(h) - shift_y_per_row * rows, 0, h - 1)
        y0 = np.floor(ys).astype(np.intp)
        y1 = np.minimum(y0 + 1, h - 1)
        fy = (ys - y0).astype(img.dtype)[:, None]
        img = img[y0, :] * (1 - fy) + img[y1, :] * fy
    if shift_x_per_row:
        xs = np.clip(np.arange(w)[None, :] - (shift_x_per_row * rows)[:, None], 0, w - 1)
        x0 = np.floor(xs).astype(np.intp)
        x1 = np.minimum(x0 + 1, w - 1)
        fx = (xs - x0).astype(img.dtype)
        img = np.take_along_axis(img, x0, 1) * (1 - fx) + np.take_along_axis(img, x1, 1) * fx
    return img


def _distortion_map(h, w, focal_render, coeffs, render_shape=None):
    """Source pixel in the rendered pinhole image for every pixel of the real (distorted) image.

    h, w: size of the real image; render_shape: (h, w) of the rendered image with
    focal length focal_render (default: same size). coeffs: k1, k2, k3, p1, p2 and
    the real camera's fx, fy, cx, cy in pixels (an OpenCV calibration at the stream
    resolution). The Brown model is inverted by fixed point iteration, like OpenCV's
    undistortPoints. Returns source x, y, a mask of pixels with a valid ray inside
    the render, and the fraction of pixels whose ray lies outside the render.
    """
    rh, rw = render_shape or (h, w)
    k1, k2, k3, p1, p2 = (float(coeffs.get(k) or 0.0) for k in ('k1', 'k2', 'k3', 'p1', 'p2'))
    fx = float(coeffs.get('fx') or focal_render * w / rw)
    fy = float(coeffs.get('fy') or fx)
    cx = float(coeffs.get('cx') if coeffs.get('cx') is not None else (w - 1) / 2.0)
    cy = float(coeffs.get('cy') if coeffs.get('cy') is not None else (h - 1) / 2.0)

    def distort(x, y):
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        return (x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
                y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y)

    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    xd, yd = (u - cx) / fx, (v - cy) / fy
    x, y = xd.copy(), yd.copy()
    with np.errstate(all='ignore'):
        for _ in range(20):
            dx, dy = distort(x, y)
            x, y = x + (xd - dx), y + (yd - dy)
        ex, ey = distort(x, y)
        converged = np.hypot(ex - xd, ey - yd) < 1e-4
    xs = x * focal_render + (rw - 1) / 2.0
    ys = y * focal_render + (rh - 1) / 2.0
    inside = (xs >= 0) & (xs <= rw - 1) & (ys >= 0) & (ys <= rh - 1)
    outside_fraction = float((converged & ~inside).mean())
    valid = converged & inside
    xs = np.where(valid, xs, 0.0)
    ys = np.where(valid, ys, 0.0)
    return np.clip(xs, 0, rw - 1), np.clip(ys, 0, rh - 1), valid, outside_fraction


def _remap(img, xs, ys):
    x0 = np.floor(xs).astype(np.intp)
    y0 = np.floor(ys).astype(np.intp)
    x1 = np.minimum(x0 + 1, img.shape[1] - 1)
    y1 = np.minimum(y0 + 1, img.shape[0] - 1)
    fx = (xs - x0).astype(img.dtype)
    fy = (ys - y0).astype(img.dtype)
    top = img[y0, x0] * (1 - fx) + img[y0, x1] * fx
    bottom = img[y1, x0] * (1 - fx) + img[y1, x1] * fx
    return top * (1 - fy) + bottom * fy


class CameraModel:
    def __init__(self, config, seed=0):
        self.cfg = dict(DEFAULTS)
        self.cfg.update(config or {})
        self.rng = np.random.default_rng(seed)
        self.shape = None
        self.exposure = 1.0     # relative exposure time (auto exposure state)
        self.gain = 1.0         # analog gain
        self.ae_started = False
        # render value -> linear light
        self.linear_lut = ((np.arange(256) / 255.0) ** self.cfg['gamma']).astype(np.float32)

    def _init_maps(self, render_h, render_w):
        c = self.cfg
        self.shape = (render_h, render_w)
        w, h = c['stream_size'] or (render_w, render_h)
        focal_render = (render_w / 2.0) / math.tan(c['horizontal_fov'] / 2.0)
        # focal length of the streamed image [px], for motion blur and rolling shutter
        self.focal_px = float(c['distortion'].get('fx') or 0.0) if c['distortion'] else 0.0
        self.focal_px = self.focal_px or focal_render * w / render_w
        # a much finer render is low-pass filtered before resampling (no aliasing):
        # sigma about 0.4 x ratio, built from [1 2 1] passes (variance 0.5 each). The
        # ratio is render focal / stream focal (render pixels per stream pixel)
        ratio = focal_render / self.focal_px
        self.prefilter_passes = max(1, int(round((0.4 * ratio) ** 2 / 0.5))) if ratio >= 1.5 else 0
        self.prnu = (1.0 + self.rng.normal(0.0, c['prnu_std'], (h, w))).astype(np.float32) \
            if c['prnu_std'] else None
        offset = np.zeros((h, w), np.float32)
        if c['dsnu_std_dn']:
            offset += self.rng.normal(0.0, c['dsnu_std_dn'] / 255.0, (h, w))
        if c['column_noise_dn']:
            offset += self.rng.normal(0.0, c['column_noise_dn'] / 255.0, (1, w))
        self.offset = offset if (c['dsnu_std_dn'] or c['column_noise_dn']) else None
        self.distortion = None
        self.distortion_outside = 0.0
        if c['distortion']:
            xs, ys, self.distortion_valid, self.distortion_outside = \
                _distortion_map(h, w, focal_render, c['distortion'], (render_h, render_w))
            self.distortion = (xs, ys)
        elif (h, w) != (render_h, render_w):
            # no lens model: plain resampling to the stream size (pixel centres)
            v, u = np.mgrid[0:h, 0:w].astype(np.float64)
            self.distortion = (np.clip((u + 0.5) * render_w / w - 0.5, 0, render_w - 1),
                               np.clip((v + 0.5) * render_h / h - 0.5, 0, render_h - 1))
            self.distortion_valid = np.ones((h, w), bool)
        self.vignette = None
        if c['vignetting']:
            v1, v2 = (list(c['vignetting']) + [0.0])[:2]
            y, x = np.mgrid[0:h, 0:w]
            cy, cx = (h - 1) / 2.0, (w - 1) / 2.0
            vx, vy = c['vignetting_center'] or (cx, cy)
            # r is normalized to the image corner distance, also for a shifted centre
            r2 = ((x - vx) ** 2 + (y - vy) ** 2) / (cx ** 2 + cy ** 2)
            self.vignette = np.clip(1.0 - v1 * r2 - v2 * r2 ** 2, 0.0, 1.0).astype(np.float32)

    def _auto_exposure(self, mean):
        """Exposure time first, analog gain once the exposure is at its maximum."""
        ae = self.cfg['auto_exposure']
        wanted = ae['target'] / max(mean, 1e-4)
        exposure = min(wanted, ae.get('max_exposure', 1.0))
        gain = min(max(wanted / exposure, 1.0), ae.get('max_gain', 1.0))
        if not self.ae_started:
            self.exposure, self.gain, self.ae_started = exposure, gain, True
            return
        # first order per frame, on a log scale
        a = 1.0 - math.exp(-1.0 / max(ae.get('tau_frames', 1.0), 1e-3))
        self.exposure = math.exp(math.log(self.exposure) + a * (math.log(exposure) - math.log(self.exposure)))
        self.gain = math.exp(math.log(self.gain) + a * (math.log(gain) - math.log(self.gain)))

    def process(self, frame, rates=None):
        """frame: uint8 render (h, w); rates: body angular velocity (wx, wy, wz) [rad/s] or None.
        Returns the streamed image (stream_size)."""
        c = self.cfg
        if self.shape != frame.shape:
            self._init_maps(*frame.shape)
        lin = self.linear_lut[frame]

        # Optics. Rotation about the optical x/y axes moves the scene at the image
        # centre by du = -f w_y, dv = f w_x (roll about the optical axis is not modelled).
        # Default camera looks along body +x: u (right) = -y, v (down) = -z.
        for _ in range(self.prefilter_passes):
            lin = _binomial(lin)
        if self.distortion is not None:
            lin = np.where(self.distortion_valid, _remap(lin, *self.distortion), 0.0).astype(np.float32)
        if c['defocus_sigma_px']:
            lin = _gaussian_blur(lin, c['defocus_sigma_px'])
        if rates is not None:
            if c['body_to_camera'] is not None:
                w_cam = np.asarray(c['body_to_camera'], float).reshape(3, 3) @ np.asarray(rates, float)
            else:
                w_cam = (-rates[1], -rates[2], rates[0])
            du = -self.focal_px * w_cam[1]       # px/s
            dv = self.focal_px * w_cam[0]
            if c['exposure_time_s']:
                t = c['exposure_time_s'] * self.exposure
                lin = _motion_blur(lin, du * t, dv * t)
            if c['row_time_s']:
                lin = _rolling_shutter(lin, du * c['row_time_s'], dv * c['row_time_s'])
        if self.vignette is not None:
            lin = lin * self.vignette

        # Sensor
        if c['auto_exposure']:
            self._auto_exposure(float(lin.mean()))
        signal = lin * self.exposure
        if self.prnu is not None:
            signal = signal * self.prnu
        if c['full_well_e']:
            # shot noise (Gaussian approximation of Poisson, exact enough above about
            # 20 e-, where read noise dominates anyway) and read noise in one draw
            full_well = float(c['full_well_e'])
            electrons = np.minimum(np.clip(signal, 0.0, None) * full_well, full_well)
            read = float(c['read_noise_e'] or 0.0)
            sigma = np.sqrt(electrons + read * read)
            electrons = electrons + sigma * self.rng.standard_normal(electrons.shape, dtype=np.float32)
            signal = electrons / full_well
        signal = signal * self.gain
        if self.offset is not None:
            signal = signal + self.offset
        if c['black_level_dn']:
            signal = signal + c['black_level_dn'] / 255.0

        # Output
        out = np.clip(signal, 0.0, 1.0) ** (1.0 / c['output_gamma'])
        return np.round(out * 255.0).astype(np.uint8)
