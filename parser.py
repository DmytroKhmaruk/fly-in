from pathlib import Path
from graph import Graph
from zone import Zone


class MapParser:
    def __init__(self) -> None:
        self.graph = Graph()
        self.nb_drones = 0
        self.start: str | None = None
        self.end: str | None = None

    def parse(self, filename: str) -> tuple[Graph, int, str, str]:
        self.graph = Graph()
        self.nb_drones = 0
        self.start = None
        self.end = None
        with Path(filename).open("r", encoding="utf-8") as file:
            lines = file.readlines()

        first_data = True
        for line_number, raw_line in enumerate(lines, 1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            try:
                if first_data:
                    self._parse_drone_count(line)
                    first_data = False
                elif line.startswith(("start_hub:", "end_hub:", "hub:")):
                    self._parse_zone(line)
                elif line.startswith("connection:"):
                    self._parse_connection(line)
                else:
                    raise ValueError("Unknown line format")

            except ValueError as error:
                raise ValueError(f"Line {line_number}: {error}") from error

        if first_data:
            raise ValueError("Missing nb_drones")

        if self.start is None or self.end is None:
            raise ValueError("Map must have one start and one end")

        return (self.graph, self.nb_drones, self.start, self.end)

    def _parse_drone_count(self, line: str) -> None:
        parts = line.split()
        if len(parts) != 2 or parts[0] != "nb_drones:":
            raise ValueError("Expected nb_drones first")
        if not parts[1].isascii() or not parts[1].isdecimal():
            raise ValueError("Invalid drone count")

        count = int(parts[1])
        if count <= 0:
            raise ValueError("Drone count must be positive")
        self.nb_drones = count

    def _split_metadata(self, content: str) -> tuple[str, str]:
        if "[" not in content:
            if "]" in content:
                raise ValueError("Unexpected closing bracket")
            return content.strip(), ""

        main, metadata = content.split("[", 1)
        if (
            not metadata.endswith("]") or "[" in metadata
            or "]" in main or "]" in metadata[:-1]
        ):
            raise ValueError("Invalid metadata brackets")

        return main.strip(), metadata[:-1].strip()

    def _parse_metadata(self, text: str, allowed: set[str]) -> dict[str, str]:
        result: dict[str, str] = {}
        if not text:
            return result
        for token in text.split():
            if token.count("=") != 1:
                raise ValueError("Invalid metadata field")

            key, value = token.split("=", 1)
            if key not in allowed:
                raise ValueError(f"Unknown metadata: {key}")
            if not value:
                raise ValueError(f"Empty metadata: {key}")
            if key in result:
                raise ValueError(f"Duplicate metadata: {key}")
            result[key] = value

        return result

    def _positive_int(self, value: str, field: str) -> int:
        if not value.isascii() or not value.isdecimal():
            raise ValueError(f"Invalid {field}")

        number = int(value)
        if number <= 0:
            raise ValueError(f"{field} must be positive")

        return number

    def _parse_zone(self, line: str) -> None:
        prefix, content = line.split(":", 1)
        main, meta_text = self._split_metadata(content)
        parts = main.split()
        if len(parts) != 3:
            raise ValueError("Expected name x y")

        name, x_text, y_text = parts
        if "-" in name:
            raise ValueError("Zone name cannot contain '-'")
        try:
            x = int(x_text)
            y = int(y_text)
        except ValueError as error:
            raise ValueError("Invalid coordinates") from error

        metadata = self._parse_metadata(
            meta_text, {"zone", "color", "max_drones"})

        zone_type = metadata.get("zone", "normal")
        color = metadata.get("color", "none")
        allowed_types = {"normal", "blocked", "restricted", "priority"}
        if zone_type not in allowed_types:
            raise ValueError(f"Invalid zone type: {zone_type}")
        capacity = self._positive_int(
            metadata.get("max_drones", "1"), "max_drones")

        if prefix == "start_hub":
            if self.start is not None:
                raise ValueError("Duplicate start_hub")
        if prefix == "end_hub":
            if self.end is not None:
                raise ValueError("Duplicate end_hub")

        zone = Zone(name, x, y, zone_type, capacity, color)
        self.graph.add_zone(zone)
        if prefix == "start_hub":
            self.start = name
        elif prefix == "end_hub":
            self.end = name

    def _parse_connection(self, line: str) -> None:
        content = line.split(":", 1)[1]
        main, meta_text = self._split_metadata(content)
        names = main.split("-")
        if len(names) != 2:
            raise ValueError("Invalid connection syntax")

        first, second = names
        if not first or not second:
            raise ValueError("Missing connection endpoint")

        metadata = self._parse_metadata(
            meta_text, {"max_link_capacity"})

        capacity = self._positive_int(
            metadata.get("max_link_capacity", "1"), "max_link_capacity")

        self.graph.add_connection(first, second, capacity)
