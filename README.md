# README

## Environments
* Ubuntu 18.04
* ROS melodic

## How to build

```bash
  $ mkdir -p ws/src
  $ cd ws
  $ catkin init
  $ wget https://raw.githubusercontent.com/chikuta/premaidai_teleoperation/master/premaidai_teleoperation.rosinstall .rosinstall
  $ rosinstall .
  $ catkin build
```

## How to launch

```bash
  $ roslaunch premaidai_teleoperation teleoperation.launch use_marker:=true
```

## Refers
* [楽しく遊ぶプリメイドAI 解析メモ](https://docs.google.com/spreadsheets/d/1c6jqMwkBroCuF74viU_q7dgSQGzacbUW4mJg-957_Rs/edit#gid=2102495394)

## LICENSE
MIT