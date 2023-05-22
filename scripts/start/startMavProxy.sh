sudo chmod 777 /dev/ttyS0
source /home/araf/.bashrc
pip install mavproxy
/home/araf/.local/bin/mavproxy.py --out=tcpin:0.0.0.0:5678 --out=tcpin:127.0.0.1:5679 --daemon
