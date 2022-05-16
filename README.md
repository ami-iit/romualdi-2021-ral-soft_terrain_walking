<h1 align="center">
  Modeling of Visco-Elastic Environments for Humanoid Robot Motion Control
</h1>


<div align="center">


_G. Romualdi, S. Dafarra and D. Pucci, "Modeling of Visco-Elastic Environments for Humanoid Robot Motion Control," in
IEEE Robotics and Automation Letters, vol. 6, no. 3, pp. 4289-4296, July 2021, doi: 10.1109/LRA.2021.3067589_

</div>

<p align="center">

https://user-images.githubusercontent.com/16744101/120328989-b1249580-c2eb-11eb-9649-ebe13ad7342c.mp4

</p>

<div align="center">
  IEEE Robotics and Automation Letters
</div>

<div align="center">
  <a href="#reproducing-the-experiments"><b>Installation</b></a> |
  <a href="https://ieeexplore.ieee.org/document/9382068"><b>Paper</b></a> |
  <a href="https://www.youtube.com/watch?v=7XKQ5ZWJvYU"><b>Video</b></a>
</div>

### Reproducing the experiments
We support running the experiments via the provided Docker image.

1. Pull the docker image:
    ```console
    docker pull ghcr.io/ami-iit/soft-terrain-walking-docker:latest
    ```
2. Launch the container:
    ```console
    xhost +
    docker run -it --rm  \
               --device=/dev/dri:/dev/dri \
               --user user \
               --env="DISPLAY=$DISPLAY"  \
               --net=host \
               ghcr.io/ami-iit/soft-terrain-walking-docker:latest
    ```
3. The application will start automatically

You can find the generated `dataset` and `images` in the `~\experiment` folder of the container.

⚠️  If you want to replicate the installation on your PC please follow the [Docker recipe](./dockerfiles/Dockerfile).

### Citing this work

If you find the work useful, please consider citing:

```bibtex
@ARTICLE{9382068,
  author={Romualdi, Giulio and Dafarra, Stefano and Pucci, Daniele},
  journal={IEEE Robotics and Automation Letters},
  title={Modeling of Visco-Elastic Environments for Humanoid Robot Motion Control},
  year={2021},
  volume={6},
  number={3},
  pages={4289-4296},
  doi={10.1109/LRA.2021.3067589}}
```

### Maintainer

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/GiulioRomualdi.png" width="40">](https://github.com/GiulioRomualdi) | [@GiulioRomualdi](https://github.com/GiulioRomualdi) |
