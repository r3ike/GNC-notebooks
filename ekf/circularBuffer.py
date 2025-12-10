import collections

# valutare se bisogna salvarsi anche acc e gyro

class CircularBuffer:
    def __init__(self, length):
        self.length = length
        self.buffer = collections.deque(maxlen=length)

    def add(self, element):
        self.buffer.append(element)
    
    
    #get last element
    def get(self, index=None):
        if not self.buffer:
            return None
        if index is None:
            return self.buffer[-1]
        if -len(self.buffer) <= index < len(self.buffer):
            return self.buffer[index]
        raise IndexError("Index out of range in CircularBuffer.")
    
    def get_by_timestamp(self, timestamp):
        idx = min(range(len(self.buffer)), key=lambda i: abs(self.buffer[i]["timestamp"] - timestamp))
        return self.buffer[idx], idx
    
    def get_last_timestamp(self):
        return self.get()["timestamp"]
    
    def update(self, index , element):
        self.buffer[index] = element

    
    def __str__(self):
        return self.buffer.__str__()



##buffer = CircularBuffer(10)

#for i in range(40):
#    buffer.add(i)
#    print(buffer)

