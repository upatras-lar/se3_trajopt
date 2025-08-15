import numpy as np


class TerrainGrid:
    def __init__(self, rows, cols, mu, min_x, min_y, max_x, max_y):
        self.rows = rows
        self.cols = cols
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.eps = 1e-6
        self.mu = mu
        self.grid = np.zeros((rows, cols))

    def out_of_bounds(self, x, y):
        return x <= self.min_x or x >= self.max_x or y <= self.min_y or y >= self.max_y

    def set_grid(self, new_grid):
        self.grid = np.array(new_grid).reshape((self.rows, self.cols))

    def set_zero(self):
        self.grid.fill(0.0)

    def height(self, x, y):
        if self.out_of_bounds(x, y):
            return 0.0

        x_norm = np.clip(
            self.rows * (x - self.min_x) / (self.max_x - self.min_x),
            0.0,
            self.rows - 1.0,
        )
        y_norm = np.clip(
            self.cols * (y - self.min_y) / (self.max_y - self.min_y),
            0.0,
            self.cols - 1.0,
        )

        x1 = np.floor(x_norm)
        x2 = np.ceil(x_norm)
        y1 = np.floor(y_norm)
        y2 = np.ceil(y_norm)

        fx1y1 = self.grid[int(x1), int(y1)]
        fx1y2 = self.grid[int(x1), int(y2)]
        fx2y1 = self.grid[int(x2), int(y1)]
        fx2y2 = self.grid[int(x2), int(y2)]

        if x1 == x2:
            fxy1 = fx1y1
            fxy2 = fx1y2
        else:
            fxy1 = ((x2 - x_norm) / (x2 - x1)) * fx1y1 + (
                (x_norm - x1) / (x2 - x1)
            ) * fx2y1
            fxy2 = ((x2 - x_norm) / (x2 - x1)) * fx1y2 + (
                (x_norm - x1) / (x2 - x1)
            ) * fx2y2

        if y1 == y2:
            height = fxy1
        else:
            height = ((y2 - y_norm) / (y2 - y1)) * fxy1 + (
                (y_norm - y1) / (y2 - y1)
            ) * fxy2

        if np.isnan(height):
            height = 0.0

        return height

    def dx_dheight(self, x, y):
        if self.out_of_bounds(x, y):
            return 0.0

        x_norm = np.clip(
            self.rows * (x - self.min_x) / (self.max_x - self.min_x),
            0.0,
            self.rows - 1.0,
        )
        y_norm = np.clip(
            self.cols * (y - self.min_y) / (self.max_y - self.min_y),
            0.0,
            self.cols - 1.0,
        )

        x1 = np.floor(x_norm)
        x2 = np.ceil(x_norm)
        y1 = np.floor(y_norm)
        y2 = np.ceil(y_norm)

        fx1y1 = self.grid[int(x1), int(y1)]
        fx1y2 = self.grid[int(x1), int(y2)]
        fx2y1 = self.grid[int(x2), int(y1)]
        fx2y2 = self.grid[int(x2), int(y2)]

        dR1dx_norm = 0.0 if x1 == x2 else (-fx1y1 + fx2y1) / (x2 - x1)
        dR2dx_norm = 0.0 if x1 == x2 else (-fx1y2 + fx2y2) / (x2 - x1)

        dfdx_norm = (
            dR1dx_norm
            if y1 == y2
            else (dR1dx_norm * (y2 - y_norm) + dR2dx_norm * (y_norm - y1)) / (y2 - y1)
        )

        dx_normdx = self.rows / (self.max_x - self.min_x)

        return dfdx_norm * dx_normdx

    def dy_dheight(self, x, y):
        if self.out_of_bounds(x, y):
            return 0.0

        x_norm = np.clip(
            self.rows * (x - self.min_x) / (self.max_x - self.min_x),
            0.0,
            self.rows - 1.0,
        )
        y_norm = np.clip(
            self.cols * (y - self.min_y) / (self.max_y - self.min_y),
            0.0,
            self.cols - 1.0,
        )

        x1 = np.floor(x_norm)
        x2 = np.ceil(x_norm)
        y1 = np.floor(y_norm)
        y2 = np.ceil(y_norm)

        fx1y1 = self.grid[int(x1), int(y1)]
        fx1y2 = self.grid[int(x1), int(y2)]
        fx2y1 = self.grid[int(x2), int(y1)]
        fx2y2 = self.grid[int(x2), int(y2)]

        R1 = (
            fx1y1
            if x1 == x2
            else (fx1y1 * (x2 - x_norm) + fx2y1 * (x_norm - x1)) / (x2 - x1)
        )
        R2 = (
            fx1y2
            if x1 == x2
            else (fx1y2 * (x2 - x_norm) + fx2y2 * (x_norm - x1)) / (x2 - x1)
        )

        dfdy_norm = 0.0 if y1 == y2 else (-R1 + R2) / (y2 - y1)

        dy_normdy = self.cols / (self.max_y - self.min_y)

        return dfdy_norm * dy_normdy

    def dxx_dheight(self, x, y):
        return 0.0

    def dyy_dheight(self, x, y):
        return 0.0

    def dxy_dheight(self, x, y):
        if self.out_of_bounds(x, y):
            return 0.0

        x_norm = np.clip(
            self.rows * (x - self.min_x) / (self.max_x - self.min_x),
            0.0,
            self.rows - 1.0,
        )
        y_norm = np.clip(
            self.cols * (y - self.min_y) / (self.max_y - self.min_y),
            0.0,
            self.cols - 1.0,
        )

        x1 = np.floor(x_norm)
        x2 = np.ceil(x_norm)
        y1 = np.floor(y_norm)
        y2 = np.ceil(y_norm)

        if x1 == x2 or y1 == y2:
            return 0.0

        fx1y1 = self.grid[int(x1), int(y1)]
        fx1y2 = self.grid[int(x1), int(y2)]
        fx2y1 = self.grid[int(x2), int(y1)]
        fx2y2 = self.grid[int(x2), int(y2)]

        dR1dx_norm = (-fx1y1 + fx2y1) / (x2 - x1)
        dR2dx_norm = (-fx1y2 + fx2y2) / (x2 - x1)

        dfdx_normy_norm = (-dR1dx_norm + dR2dx_norm) / (y2 - y1)

        dy_normdy = self.cols / (self.max_y - self.min_y)
        dx_normdx = self.rows / (self.max_x - self.min_x)

        return dfdx_normy_norm * dy_normdy * dx_normdx

    def dyx_dheight(self, x, y):
        return self.dxy_dheight(x, y)

    def n(self, x, y):
        dx = self.dx_dheight(x, y)
        dy = self.dy_dheight(x, y)
        v = np.array([[-dx, -dy, 1.0]]).T
        return v / np.linalg.norm(v)

    def t1(self, x, y):
        dx = self.dx_dheight(x, y)
        v = np.array([[1.0, 0.0, dx]]).T
        return v / np.linalg.norm(v)

    def t2(self, x, y):
        dy = self.dy_dheight(x, y)
        v = np.array([[0.0, 1.0, dy]]).T
        return v / np.linalg.norm(v)

    def dx_dn(self, x, y, eps=1e-6):
        I = np.identity(3)

        dx = self.dx_dheight(x, y)
        dy = self.dy_dheight(x, y)
        vec = np.array([[-dx, -dy, 1.0]]).T

        dxx = self.dxx_dheight(x, y)
        dyx = self.dyx_dheight(x, y)
        dvec = np.array([[-dxx, -dyx, 0.0]]).T

        norm = np.linalg.norm(vec)
        tmp = (1 / norm) * (I - ((vec @ vec.T) / (norm**2)))
        deriv = tmp @ dvec
        return deriv

    def dy_dn(self, x, y, eps=1e-6):
        I = np.identity(3)

        dx = self.dx_dheight(x, y)
        dy = self.dy_dheight(x, y)
        vec = np.array([[-dx, -dy, 1.0]]).T

        dxy = self.dxy_dheight(x, y)
        dyy = self.dyy_dheight(x, y)
        dvec = np.array([[-dxy, -dyy, 0.0]]).T

        norm = np.linalg.norm(vec)
        tmp = (1 / norm) * (I - ((vec @ vec.T) / (norm**2)))
        deriv = tmp @ dvec
        return deriv

    def dx_dt1(self, x, y, eps=1e-6):
        I = np.identity(3)

        dx = self.dx_dheight(x, y)
        vec = np.array([[1.0, 0.0, dx]]).T

        dxx = self.dxx_dheight(x, y)
        dvec = np.array([[0.0, 0.0, dxx]]).T

        norm = np.linalg.norm(vec)
        tmp = (1 / norm) * (I - ((vec @ vec.T) / (norm**2)))
        deriv = tmp @ dvec
        return deriv

    def dy_dt1(self, x, y, eps=1e-6):
        I = np.identity(3)

        dx = self.dx_dheight(x, y)
        vec = np.array([[1.0, 0.0, dx]]).T

        dxy = self.dxy_dheight(x, y)
        dvec = np.array([[0.0, 0.0, dxy]]).T

        norm = np.linalg.norm(vec)
        tmp = (1 / norm) * (I - ((vec @ vec.T) / (norm**2)))
        deriv = tmp @ dvec
        return deriv

    def dx_dt2(self, x, y, eps=1e-6):
        I = np.identity(3)

        dy = self.dy_dheight(x, y)
        vec = np.array([[0.0, 1.0, dy]]).T

        dyx = self.dyx_dheight(x, y)
        dvec = np.array([[0.0, 0.0, dyx]]).T

        norm = np.linalg.norm(vec)
        tmp = (1 / norm) * (I - ((vec @ vec.T) / (norm**2)))
        deriv = tmp @ dvec
        return deriv

    def dy_dt2(self, x, y, eps=1e-6):
        I = np.identity(3)

        dy = self.dy_dheight(x, y)
        vec = np.array([[0.0, 1.0, dy]]).T

        dyy = self.dyy_dheight(x, y)
        dvec = np.array([[0.0, 0.0, dyy]]).T

        norm = np.linalg.norm(vec)
        tmp = (1 / norm) * (I - ((vec @ vec.T) / (norm**2)))
        deriv = tmp @ dvec
        return deriv
