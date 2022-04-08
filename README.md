# swarm-hashgraph
This repository is an implementation of the framework for hashgraph-based swarm robotics.
There are two case studies in this repository, one is black-white ratio estimation and the other is object searching
## Requirements
Ubuntu18.04, Webots 2021a
## Install
###
    git clone https://github.com/luoxiaojie9527/swarm-hashgraph.git
## Usage
Take object-searching for example
1. Launch the Hashgrpah process
###
    cd object_searching/controllers/py-swirld-object-searching/
    python swirldsRestart.py    
2. Launch the Webots simulator by click the webots icon
3. In Webots simulator, open the world object_searching2.wbt in object_searching/worlds/
4. Click the run button
## Tips
Don't forget to change the absolute path of the document in the code to read your preset parameters and save your experimental results.
