from abc import ABCMeta, abstractmethod

class Factory(metaclass=ABCMeta):
    count = 0
    
    def __init__(self):
        Factory.count += 1
        self.action_id = Factory.count

    @abstractmethod
    def echo(self):
        pass

class Afact(Factory):
    def __init__(self, n):
        super().__init__()
        self.n = n
    
    def echo(self):
        print(self.action_id, self.n)

class Bfact(Factory):
    def __init__(self, n):
        super().__init__()
        self.n = n
    
    def echo(self):
        print(self.action_id, self.n)

a = Afact('this is a')
b = Bfact('this is b')
a.echo()
b.echo()
print(b.count, a.count, Factory.count)
    