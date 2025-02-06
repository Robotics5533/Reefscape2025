class HashMap:
    def __init__(self):
        self.hash_map = {}

    def get(self, key: str):
        if key in self.hash_map:
            return self.hash_map[key]
        else:
            return None

    def put(self, key: str, value: str):
        self.hash_map[key] = value

    def size(self):
        return len(self.hash_map)

    def remove(self, key: str):
        if self.contains_key(key):
            del self.hash_map[key]

    def contains_key(self, key: str):
        return key in self.hash_map

    def is_empty(self):
        return len(self.hash_map) == 0
