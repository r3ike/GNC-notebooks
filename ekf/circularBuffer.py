import collections

class CircularBuffer:
    def __init__(self, length):
        self.length = length
        self.buffer = collections.deque(maxlen=length)

    def add(self, element):
        self.buffer.append(element)
    
    def __str__(self):
        return self.buffer.__str__()



##buffer = CircularBuffer(10)

#for i in range(40):
#    buffer.add(i)
#    print(buffer)

