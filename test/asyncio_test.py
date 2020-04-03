import asyncio

class A():
    def __init__(self):
        self.t = 0

async def disp():
    loop = asyncio.get_running_loop()
    a = A()
    endtime = loop.time()+5.0
    while True:
        print(a.t)
        if a.t > 100000:
            break
        await asyncio.sleep(0.001)

def main():
    asyncio.run(disp())

if __name__ == '__main__':
    main()