import math

class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(self, dt: float, lookahead_distance: float = 0.5):
        """Pure pursuit class initializer.

        Args:
            dt: Sampling period [s].
            lookahead_distance: Distance to the next target point [m].

        """
        self._dt: float = dt
        self._lookahead_distance: float = lookahead_distance
        self._path: list[tuple[float, float]] = []

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """Pure pursuit controller implementation.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
            theta: Estimated robot heading [rad].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 4.4. Complete the function body with your code (i.e., compute v and w).
        # v = 0.0
        # w = 0.0
        
        if not self._path:
            return 0.0, 0.0

        # # 1. Encontrar punto más cercano y luego el target
        # _, closest_idx = self._find_closest_point(x, y)
        # tx, ty = self._find_target_point((x, y), closest_idx)

        # # 2. Calcular ángulo hacia el target (alpha)
        # angle_to_target = math.atan2(ty - y, tx - x)
        # alpha = angle_to_target - theta
        
        # # Normalizar alpha a [-pi, pi]
        # alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # # 3. Calcular curvatura y comandos
        # # L = distancia real al target
        # L = math.hypot(tx - x, ty - y)
        
        # v = 0.5  # Velocidad lineal constante (ejemplo)
        
        # # w = v * (2 * sin(alpha) / L)
        # if L > 0:
        #     w = v * (2.0 * math.sin(alpha) / L)
        # else:
        #     w = 0.0
            
        _, closest_idx = self._find_closest_point(x, y)
        tx, ty = self._find_target_point((x, y), closest_idx)

        # Lógica de Pure Pursuit
        angle_to_target = math.atan2(ty - y, tx - x)
        alpha = math.atan2(math.sin(angle_to_target - theta), math.cos(angle_to_target - theta))
        L = math.hypot(tx - x, ty - y)
        
        v = 0.3  # Velocidad recomendada para pruebas iniciales
        w = v * (2.0 * math.sin(alpha) / L) if L > 0 else 0.0
        
        return v, w

    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value

    def _find_closest_point(self, x: float, y: float) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        # TODO: 4.2. Complete the function body (i.e., find closest_xy and closest_idx).
        # closest_xy = (0.0, 0.0)
        # closest_idx = 0
        
        if not self._path:
            return (x, y), 0

        # Buscamos el punto con la distancia mínima
        closest_idx = min(
            range(len(self._path)),
            key=lambda i: math.hypot(self._path[i][0] - x, self._path[i][1] - y)
        )
        closest_xy = self._path[closest_idx]

        return closest_xy, closest_idx
        
    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """Find the destination path point based on the lookahead distance.

        Args:
            origin_xy: Current location of the robot (x, y) [m].
            origin_idx: Index of the current path point.

        Returns:
            tuple[float, float]: (x, y) coordinates of the target point [m].

        """
        # TODO: 4.3. Complete the function body with your code (i.e., determine target_xy).
        #target_xy = (0.0, 0.0)
        
        if not self._path:
            return origin_xy

        target_xy = self._path[-1] # Por defecto, el final de la ruta
        
        # Buscamos el primer punto que supere la distancia de seguridad (lookahead)
        for i in range(origin_idx, len(self._path)):
            dist = math.hypot(self._path[i][0] - origin_xy[0], 
                              self._path[i][1] - origin_xy[1])
            if dist >= self._lookahead_distance:
                target_xy = self._path[i]
                break

        return target_xy
        