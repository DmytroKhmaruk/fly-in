class Connection:
    def __init__(self, first: str, second: str, max_link_capacity: int = 1
                 ) -> None:
        self.first = first
        self.second = second
        self.max_link_capacity = max_link_capacity
