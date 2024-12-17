execute with

```
docker build -t orbslam3ros .
```

```
xhost +local:docker
```


```
docker run --network host --privileged -v /dev:/dev orbslam3ros
```