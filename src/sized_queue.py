from multiprocessing import Queue

class SizedQueue():
    def __init__(self, size):
        self.queue = Queue(size)
    
    def put(self, value):
        if self.queue.full():
            self.queue.get()
        self.queue.put(value)

    def get(self):
        return self.queue.get()

    def full(self):
        return self.queue.full()