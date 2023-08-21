# Installing igibson simulator
Detailed instructions can be found [here](https://stanfordvl.github.io/iGibson/installation.html)

## Install miniconda
```
curl -LO http://repo.continuum.io/miniconda/Miniconda-latest-Linux-x86_64.sh
bash Miniconda-latest-Linux-x86_64.sh
rm Miniconda-latest-Linux-x86_64.sh
```

### Add conda to your PATH
echo "export PATH=$HOME/<path_to_your_miniconda_installation_folder>/bin:$PATH" >> .bashrc

## Update conda and create a virtual environment for iGibson
conda update -y conda
conda create -y -n igibson python=3.8
conda activate igibson

## Check and link cuda version
* Check cuda version from `nvcc --version`
* symlink cuda-\<version\> to cuda by  `ln -s /usr/local/cuda-<version> /usr/local/cuda`

## Compile igibson from source
(activate the environment if not already)
```
git clone https://github.com/StanfordVL/iGibson --recursive
cd iGibson
pip install -e .
```

# Integrating with ROS
Detailed Instructions can be found [here](https://stanfordvl.github.io/iGibson/ros_integration.html)

Assuming above iGisbon has been installed at `\<iGibson root>` folder, miniconda/anaconda installed at `\<conda installation root>`,

```
export PYTHONPATH="$PYTHONPATH":<conda installation root>/env/igibson/python3.8/site-packages:<iGibson root>
```
make sure to follow the above order while updating the PYTHONAPTH variable

Assuming a workspace has already been created, 

```
cd <catkin workspace root>/src
ln -s <iGibson root>/igibson/examples/ros/igibson-ros/ .
```
If a custom version of `<igibson-ros>` is available, link it inside the src fodler rather than the default in iGibson's

``` 
source /opt/ros/noetic/setup.bash
cd .. && catkin_make
```

Install any missing dependencies
`pip install empy catkin_pkg rospkg`

`rosdep install --from-paths src --ignore-src -r -y`
