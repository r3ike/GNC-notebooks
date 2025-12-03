import collections

# valutare se bisogna salvarsi anche acc e gyro

class CircularBuffer:
    def __init__(self, length):
        self.length = length
        self.buffer = collections.deque(maxlen=length)

    def add(self, element):
        self.buffer.append(element)
    
    def add(self, timestamp, x_nominal, P):
        self.buffer.append({

        })
    
    def get(self, index):
        return self.buffer[index]
    
    def update(self, index , new_element):
        self.buffer[index] = new_element
    
    def __str__(self):
        return self.buffer.__str__()



##buffer = CircularBuffer(10)

#for i in range(40):
#    buffer.add(i)
#    print(buffer)

