import math
import re
import pygame
from drone import Drone
from graph import Graph
from simulation import Simulation


class DroneGUI:
    WIDTH = 1100
    HEIGHT = 750
    FPS = 60
    BACKGROUND = (15, 23, 42)
    WHITE = (241, 245, 249)
    MUTED = (148, 163, 184)
    EDGE = (71, 85, 105)
    CYAN = (34, 211, 238)
    GREEN = (52, 211, 153)
    COLORS = {
        "normal": (59, 130, 246),
        "restricted": (239, 68, 68),
        "priority": (34, 197, 94),
        "blocked": (100, 116, 139),
    }

    def __init__(self, graph: Graph, simulation: Simulation, start: str,
                 end: str) -> None:
        self.graph = graph
        self.simulation = simulation
        self.start = start
        self.end = end
        self.initial_drones = [
            (drone.drone_id, drone.path.copy())
            for drone in simulation.drones]

        pygame.init()
        self.screen = pygame.display.set_mode(
            (self.WIDTH, self.HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption("Fly-in | Drone Simulation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("arial", 18)
        self.small_font = pygame.font.SysFont("arial", 13)
        self.big_font = pygame.font.SysFont("arial", 26, bold=True)

        zones = list(self.graph.zones.values())
        if not zones:
            raise ValueError("The map contains no zones")
        self.min_x = min(zone.x for zone in zones)
        self.max_x = max(zone.x for zone in zones)
        self.min_y = min(zone.y for zone in zones)
        self.max_y = max(zone.y for zone in zones)
        self.world_center = (
            (self.min_x + self.max_x) / 2,
            (self.min_y + self.max_y) / 2)

        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.dragging = False
        self.running = True
        self.paused = False
        self.turn_duration = 1.0
        self.progress = 1.0
        self.history = [self._snapshot()]
        self.current_turn = 0
        self.error_message = ""

    def _scale(self) -> float:
        width, height = self.screen.get_size()
        world_width = max(self.max_x - self.min_x, 1)
        world_height = max(self.max_y - self.min_y, 1)
        return min(
            max(width - 170, 80) / world_width,
            max(height - 230, 80) / world_height) * self.zoom

    def _screen_point(self, x: float, y: float) -> tuple[float, float]:
        width, height = self.screen.get_size()
        scale = self._scale()
        return (
            width / 2 + self.pan_x + (x - self.world_center[0]) * scale,
            height / 2 + 15 + self.pan_y
            + (y - self.world_center[1]) * scale)

    def _zoom_at(self, mouse: tuple[int, int], factor: float) -> None:
        new_zoom = max(0.25, min(8.0, self.zoom * factor))
        ratio = new_zoom / self.zoom
        width, height = self.screen.get_size()
        anchor_x = width / 2
        anchor_y = height / 2 + 15
        self.pan_x = (
            mouse[0] - anchor_x
            - (mouse[0] - anchor_x - self.pan_x) * ratio)
        self.pan_y = (
            mouse[1] - anchor_y
            - (mouse[1] - anchor_y - self.pan_y) * ratio)
        self.zoom = new_zoom

    def _fit_map(self) -> None:
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0

    def _drone_world_position(self, drone: Drone) -> tuple[float, float]:
        origin = self.graph.zones[drone.current_zone]
        x, y = float(origin.x), float(origin.y)
        if drone.in_transit:
            target = self.graph.zones[drone.path[drone.position + 1]]
            x = (x + target.x) / 2
            y = (y + target.y) / 2
        return x, y

    def _snapshot(self) -> dict[int, tuple[float, float, bool, bool]]:
        result = {}
        for drone in self.simulation.drones:
            x, y = self._drone_world_position(drone)
            result[drone.drone_id] = (
                x, y, drone.finished,
                drone.current_zone == self.start and not drone.in_transit)
        return result

    def _drone_offsets(self, frame: dict[int, tuple[float, float, bool, bool]]
                       ) -> dict[int, tuple[float, float]]:
        groups: dict[tuple[float, float], list[int]] = {}
        for drone_id, state in frame.items():
            x, y, finished, _ = state
            if finished:
                continue

            groups.setdefault((x, y), []).append(drone_id)

        offsets: dict[int, tuple[float, float]] = {}
        for drone_ids in groups.values():
            drone_ids.sort()
            count = len(drone_ids)

            if count == 1:
                offsets[drone_ids[0]] = (0.0, 0.0)
                continue

            radius = max(18.0, 12.0 * count / math.pi)
            for index, drone_id in enumerate(drone_ids):
                angle = 2 * math.pi * index / count
                offsets[drone_id] = (math.cos(angle) * radius, math.sin(angle)
                                     * radius)

        return offsets

    def _is_complete(self) -> bool:
        return all(
            state[2] for state in self.history[self.current_turn].values())

    def _next_turn(self, animated: bool = False) -> None:
        if self.current_turn + 1 < len(self.history):
            self.current_turn += 1
        else:
            if self._is_complete() or self.error_message:
                return
            try:
                movements = self.simulation.step()
                if movements:
                    print(" ".join(movements), flush=True)
            except (RuntimeError, ValueError) as error:
                self.error_message = str(error)
                self.paused = True
                return
            self.history.append(self._snapshot())
            self.current_turn += 1
        self.progress = 0.0 if animated else 1.0

    def _previous_turn(self) -> None:
        if self.current_turn > 0:
            self.current_turn -= 1
        self.progress = 1.0
        self.paused = True

    def _restart(self) -> None:
        drones = [
            Drone(drone_id, path.copy())
            for drone_id, path in self.initial_drones
        ]
        self.simulation = Simulation(self.graph, drones)
        self.history = [self._snapshot()]
        self.current_turn = 0
        self.progress = 1.0
        self.paused = True
        self.error_message = ""

    @staticmethod
    def _short_name(name: str) -> str:
        if len(name) <= 7:
            return name
        words = [word for word in name.split("_") if word]
        if len(words) > 1:
            suffix = re.search(r"\d+$", name)
            prefix = "".join(word[0].upper() for word in words[:3])
            result = prefix + (suffix.group() if suffix else "")
            return result[:7]
        return name[:6] + "…"

    def _draw_connections(self) -> None:
        for connection in self.graph.connections.values():
            first = self.graph.zones[connection.first]
            second = self.graph.zones[connection.second]
            a = self._screen_point(first.x, first.y)
            b = self._screen_point(second.x, second.y)
            pygame.draw.line(self.screen, self.EDGE, a, b, 2)

    def _draw_zones(self) -> None:
        for name, zone in self.graph.zones.items():
            x, y = self._screen_point(zone.x, zone.y)
            center = (round(x), round(y))
            color = self.COLORS.get(zone.zone_type, self.EDGE)

            if name == self.start:
                color = (22, 163, 74)
            elif name == self.end:
                color = (245, 158, 11)

            if zone.color != "none":
                try:
                    custom_color = pygame.Color(zone.color)
                    color = (custom_color.r, custom_color.g, custom_color.b)
                except ValueError:
                    pass

            pygame.draw.circle(self.screen, color, center, 21)
            pygame.draw.circle(self.screen, self.WHITE, center, 21, 1)
            text = self.small_font.render(
                self._short_name(name), True, self.WHITE
            )
            self.screen.blit(text, text.get_rect(center=center))

        finished = sum(
            state[2] for state in self.history[self.current_turn].values()
        )
        total = len(self.history[self.current_turn])
        end_zone = self.graph.zones[self.end]
        x, y = self._screen_point(end_zone.x, end_zone.y)
        delivered = self.small_font.render(
            f"{finished}/{total} delivered", True, self.GREEN
        )
        self.screen.blit(delivered, (round(x) + 27, round(y) - 9))

    def _draw_drones(self) -> None:
        current = self.history[self.current_turn]
        previous = self.history[max(0, self.current_turn - 1)]
        current_offsets = self._drone_offsets(current)
        previous_offsets = self._drone_offsets(previous)
        for drone_id, state in current.items():
            x1, y1, finished, _ = state
            if finished and self.progress >= 1.0:
                continue

            x0, y0, _, _ = previous[drone_id]
            x0, y0 = self._screen_point(x0, y0)
            x1, y1 = self._screen_point(x1, y1)
            dx0, dy0 = previous_offsets.get(drone_id, (0.0, 0.0))
            dx1, dy1 = current_offsets.get(drone_id, (0.0, 0.0))
            x0 += dx0
            y0 += dy0
            x1 += dx1
            y1 += dy1
            t = self.progress
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            center = (round(x), round(y))
            pygame.draw.circle(self.screen, self.CYAN, center, 10)
            pygame.draw.circle(self.screen, self.WHITE, center, 10, 1)
            label = self.small_font.render(str(drone_id), True,
                                           self.BACKGROUND)
            self.screen.blit(label, label.get_rect(center=center))

    def _draw_hover(self) -> None:
        mouse = pygame.mouse.get_pos()
        for name, zone in self.graph.zones.items():
            x, y = self._screen_point(zone.x, zone.y)
            if math.dist(mouse, (x, y)) > 24:
                continue
            caption = f"{name}  |  {zone.zone_type}"
            label = self.font.render(caption, True, self.WHITE)
            rectangle = label.get_rect()
            rectangle.topleft = (mouse[0] + 16, mouse[1] + 15)
            rectangle.inflate_ip(16, 12)
            width, height = self.screen.get_size()
            rectangle.clamp_ip(pygame.Rect(0, 0, width, height))
            pygame.draw.rect(
                self.screen, (30, 41, 59), rectangle, border_radius=7
            )
            pygame.draw.rect(
                self.screen, self.MUTED, rectangle, 1, border_radius=7
            )
            self.screen.blit(label, label.get_rect(center=rectangle.center))
            break

    def _draw_hud(self) -> None:
        height = self.screen.get_height()
        title = self.big_font.render(
            "FLY-IN | DRONE SIMULATOR", True, self.WHITE
        )
        self.screen.blit(title, (25, 20))
        states = self.history[self.current_turn]
        delivered = sum(state[2] for state in states.values())
        status = (
            f"Turn: {self.current_turn}    "
            f"Delivered: {delivered}/{len(states)}    "
            f"Speed: {1 / self.turn_duration:.1f}x    "
            f"Zoom: {self.zoom:.1f}x"
        )
        self.screen.blit(
            self.font.render(status, True, self.WHITE), (25, 65)
        )
        commands = [
            "SPACE Pause  |  LEFT/RIGHT Steps  |  UP/DOWN Speed",
            "Wheel Zoom  |  Middle/Right drag Pan  |  F Fit  |  R Restart  |  "
            "ESC Exit",
        ]
        for i, line in enumerate(commands):
            caption = self.small_font.render(line, True, self.MUTED)
            self.screen.blit(caption, (25, height - 47 + i * 18))

        message = self.error_message
        if not message:
            if self._is_complete() and self.progress >= 1.0:
                message = "ALL DRONES DELIVERED!"
            elif self.paused:
                message = "PAUSED"
        if message:
            self.screen.blit(
                self.font.render(message[:90], True, (250, 204, 21)),
                (25, height - 80),
            )

    def run(self) -> None:
        try:
            while self.running:
                dt = self.clock.tick(self.FPS) / 1000.0
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.MOUSEWHEEL:
                        self._zoom_at(
                            pygame.mouse.get_pos(),
                            1.15 ** event.y,
                        )
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button in (2, 3):
                            self.dragging = True
                    elif event.type == pygame.MOUSEBUTTONUP:
                        if event.button in (2, 3):
                            self.dragging = False
                    elif event.type == pygame.MOUSEMOTION:
                        if self.dragging:
                            self.pan_x += event.rel[0]
                            self.pan_y += event.rel[1]
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                        elif event.key == pygame.K_SPACE:
                            self.paused = not self.paused
                        elif event.key == pygame.K_UP:
                            self.turn_duration = max(
                                0.2, self.turn_duration - 0.2
                            )
                        elif event.key == pygame.K_DOWN:
                            self.turn_duration = min(
                                5.0, self.turn_duration + 0.2
                            )
                        elif event.key == pygame.K_LEFT:
                            self._previous_turn()
                        elif event.key == pygame.K_RIGHT:
                            self.paused = True
                            self._next_turn(animated=False)
                        elif event.key == pygame.K_r:
                            self._restart()
                        elif event.key == pygame.K_f:
                            self._fit_map()

                if not self.paused:
                    if self.progress < 1.0:
                        self.progress = min(
                            1.0, self.progress + dt / self.turn_duration
                        )
                    elif not self._is_complete():
                        self._next_turn(animated=True)

                self.screen.fill(self.BACKGROUND)
                self._draw_connections()
                self._draw_zones()
                self._draw_drones()
                self._draw_hover()
                self._draw_hud()
                pygame.display.flip()
        finally:
            pygame.quit()
