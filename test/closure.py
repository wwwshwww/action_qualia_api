import time

def pp(a, b):
    i1 = a
    i2 = b
    tim = time.time()
    def inner():
        if b == 5:
            return 'aaaaa!'
        else:
            return i1, i2, tim, time.time()
    return inner

def main():
    f1 = pp(1,1)
    f2 = pp(2,5)
    time.sleep(1)
    print(f2())
    print(f1())
    time.sleep(1)
    print(f1())

if __name__ == '__main__':
    main()