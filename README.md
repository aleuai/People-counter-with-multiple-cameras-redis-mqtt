# People Counter With Multiple Cameras Using Redis and MQTT

This program has two sample models: one way of streaming from your device (Raspberry, Nano, TX2, etc) using redis-serv and how you can receive and replay your analytics over two or more videos in real time.

I have been using the similar model as a double check over the single analysis and have also been testing streaming comportment using redis without analytics program on device.

On Raspberry 3B+ 2018 without Desktop OS, Python 3.9 and OpenCV, I no longer have temperature issues and memory consumption is lower than usual. I needed to use Supervisor to control the live.py program.

# Running the program

```
python3 Count.py --host cam1.host cam2.host camn.host  --w 640 --h 480 --track False --lup 3.5 --ldown 4.0 --elimit 0.3
```
