import numpy as np

class Solver:
    """
    An optimization solver for equations
    Args:
        method (str): The optimization method to use (default: "Newton").
        tol (float): The tolerance for convergence (default: 1e-6).
        max_iter (int): The maximum number of iterations (default: 100).
        function (callable): The function to optimize (default: None).
        derivative (callable): The derivative of the function (default: None).
    """
    def __init__(self, method = "Newton", tol=1e-6, max_iter=100, function=None, derivative=None):
        self.method = method
        self.tol = tol
        self.max_iter = max_iter
        self.function = function
        self.derivative = derivative
        self.method_list = ["Newton", "Bisection", "Secant", "Regula Falsi"]

    def solve(self, x0, x1=None):
        """Solve the equation using the specified method.

        Args:
            x0 (float): The initial guess or lower bound.
            x1 (float, optional): The upper bound (for methods that require it). Defaults to None.

        Raises:
            ValueError: If the method is not recognized or if the function is not defined.

        Returns:
            float: The solution found by the solver.

        Raises:
            ValueError: If the method is not recognized or if the function is not defined.

        Returns:
            float: The solution found by the solver.    
        """
        if self.method == "Newton":
            return self.newton_method(x0)
        elif self.method == "Bisection":
            return self.bisection_method(x0, x1)
        elif self.method == "Secant":
            return self.secant_method(x0, x1)
        elif self.method == "Regula Falsi":
            return self.regula_falsi_method(x0, x1)
        else:
            raise ValueError("Unknown method: {}".format(self.method))

    def newton_method(self, x0):
        for i in range(self.max_iter):
            if self.derivative is None:
                raise ValueError("Derivative is not defined.")

            x1 = x0 - self.function(x0) / self.derivative(x0)
            if abs(x1 - x0) < self.tol:
                return x1
            x0 = x1
        raise ValueError("Failed to converge.")
    
    def bisection_method(self, x0, x1=None):
        if x1 is None:
            x1 = x0 + 1
        if self.function is None:
            raise ValueError("Function is not defined.")

        for i in range(self.max_iter):
            if self.function(x0) * self.function(x1) > 0:
                raise ValueError("Function values at the endpoints must have opposite signs.")

            x2 = (x0 + x1) / 2
            if abs(x2 - x0) < self.tol:
                return x2

            if self.function(x2) == 0:
                return x2
            elif self.function(x0) * self.function(x2) < 0:
                x1 = x2
            else:
                x0 = x2

        raise ValueError("Failed to converge.")

    def secant_method(self, x0, x1):
        if self.function is None:
            raise ValueError("Function is not defined.")
        if x1 is None:
            x1 = x0 + 1

        for i in range(self.max_iter):
            if abs(x1 - x0) < self.tol:
                return x1

            x2 = x1 - self.function(x1) * (x1 - x0) / (self.function(x1) - self.function(x0))
            x0, x1 = x1, x2

        raise ValueError("Failed to converge.")

    def regula_falsi_method(self, x0, x1):
        if self.function is None:
            raise ValueError("Function is not defined.")
        if x1 is None:
            x1 = x0 + 1
        for i in range(self.max_iter):
            if self.function(x0) * self.function(x1) > 0:
                raise ValueError("Function values at the endpoints must have opposite signs.")

            x2 = x1 - self.function(x1) * (x1 - x0) / (self.function(x1) - self.function(x0))
            if abs(x2 - x1) < self.tol:
                return x2

            if self.function(x2) == 0:
                return x2
            elif self.function(x0) * self.function(x2) < 0:
                x1 = x2
            else:
                x0 = x2

        raise ValueError("Failed to converge.")

    def get_method_list(self):
        return self.method_list

if __name__ == "__main__":
    # Example usage
    # solve: x^2 = 2
    solver = Solver(
        method="Newton",
        tol=1e-6,
        max_iter=100,
        function=lambda x: x**2 - 2,
        derivative=lambda x: 2*x
    )
    root = solver.solve(1.0)
    print("Root found:", root)
    
    # example sin(a) = sqrt(2)/2
    solver = Solver(
        method="Regula Falsi",
        tol=1e-6,
        max_iter=100,
        function=lambda x: np.sin(x) - np.sqrt(2)/2,
        derivative=lambda x: np.cos(x)
    )
    root = solver.solve(np.pi/3, np.pi/8)
    print("Root found:", np.rad2deg(root))