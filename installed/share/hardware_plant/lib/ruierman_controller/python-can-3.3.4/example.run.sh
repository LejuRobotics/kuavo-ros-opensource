export LD_LIBRARY_PATH=../python-can-3.3.4/can/interfaces/bmcan:$PATH
export PYTHONPATH=.:../python-can-3.3.4:$PYTHONPATH
python3 ./examples/receive_all.py
