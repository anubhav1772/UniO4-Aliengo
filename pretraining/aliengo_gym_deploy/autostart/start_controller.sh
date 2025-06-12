#!/bin/bash
sudo docker stop foxy_controller || true
sudo docker rm foxy_controller || true
cd ~/aliengo_gym/aliengo_gym_deploy/docker/
sudo make autostart