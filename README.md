# CRA1 Advanced Augmented Reality
Duckietown CRA1 Advanced Augmented Reality Exercise

Instructions to reproduce results

### 1. Clone this repository and go to its directory
```bash
git clone https://github.com/lineojcd/CRA1_adv.git
cd CRA1_adv
```
### 2. Build docker image in Duckiebot
```bash
dts devel build -f --arch arm32v7 -H [ROBOT_NAME].local 
```

### 3. Run docker image in Duckiebot with the following options
```bash
docker -H [ROBOT_NAME].local run -it -v /data/config/calibrations/:/code/catkin_ws/src/cra1_adv/calibrations/ --rm --net=host --privileged duckietown/cra1_adv:latest-arm32v7
```
If the above command does not work, replace "CRA1_adv" by lower case ("cra1_adv")

# Credit
Credit to afdaniele for the image projection pipeline (https://github.com/duckietown/dt-core/blob/6285293fb05351a23dc936a3dd70d42e61bca379/packages/ground_projection/include/ground_projection/GroundProjection.py) and also to lib-dt-apriltags: Python bindings for the Apriltags library (https://github.com/duckietown/lib-dt-apriltags)



A sample result is shown here:
![testapriltagdetector](https://github.com/lineojcd/CRA1_adv/blob/main/testimgdector.png)
