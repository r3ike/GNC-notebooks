class CircularBuffer:
    def __init__(self, length):
        self.length = length
        self.buffer = [length]

    def add(self, element):
        self.buffer[len(self.buffer) % self.length] = element

