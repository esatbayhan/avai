echo "[Build workspace]"
source ~/.bashrc
cd /workspace/avai
pip install --upgrade pip
pip3 list --outdated --format=freeze | grep -v '^\-e' | cut -d = -f 1 | xargs -n1 pip3 install -U 
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install --upgrade torch torchvision tabulate
colcon build --symlink-install