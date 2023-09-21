# swarm-hashgraph
This repository is an implementation of the framework for hashgraph-based swarm robotics.
There are two case studies in this repository, one is black-white ratio estimation and the other is object searching.
The corresponding article: A Fast and Robust Solution for Common Knowledge Formation in Decentralized Swarm Robots.https://link.springer.com/article/10.1007/s10846-022-01759-1
## Requirements
Ubuntu18.04, Webots 2021a

Requirements of py-swirld installation refers to https://github.com/Lapin0t/py-swirld
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
## Demo Video
https://pan.baidu.com/s/1mtrY0Fwr6RJt6HVxyEfKdA?pwd=s6s5#list/path=%2F
