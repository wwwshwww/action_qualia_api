from threading import Timer

class RepeatedTimer(Timer):
    def __init__(self, interval, callback:callable, args:list=None, kwargs:dict=None):
        Timer.__init__(self, interval, self.run, args, kwargs)
        self.thread = None
        self.callback = callback

    def run(self):
        self.thread = Timer(self.interval, self.run)
        self.thread.start()
        self.callback(*self.args, **self.kwargs)

    def cancel(self):
        if self.thread is not None:
            self.thread.cancel()
            self.thread.join()
            del self.thread

if __name__=='__main__':    

    import time

    def hello(message):
        hello.counter += 1
        print(message, hello.counter)
    hello.counter = 0

    class hh():
        def __init__(self):
            self.ho = 'a'
            self.count = 0
        def pr(self, m):
            self.count += 1
            print(self.ho, m, self.count)

    h = hh()
    h.ho = 'b'
    t = RepeatedTimer(0.1, hello, ['hello'])
    t2 = RepeatedTimer(0.1, h.pr, ['hoi'])
    t.start()
    t2.start()
    time.sleep(2)
    t.cancel()
    t2.cancel()
    print("Done.", 1|1<<1)