# MayaNodes
A basic Maya deformer node to test and compare multithreading performances.
it's currently WIP and while the deformation is working, it's not in parallel

There's a MultiThreadingType parameter:
0 = no multithreading

1= MThreadPool -> should work but isn't

2= TBB -> should work but isn't

3= OMP -> the cpu go up to 100% but performance are exactly the same as single threaded
