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
    
    def update(self, index , x_nominal_new, P_new):
        self.buffer[index].x_nominal = x_nominal_new
        self.buffer[index].P = P_new

    
    def __str__(self):
        return self.buffer.__str__()



##buffer = CircularBuffer(10)

#for i in range(40):
#    buffer.add(i)
#    print(buffer)

