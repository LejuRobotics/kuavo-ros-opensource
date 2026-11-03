export LD_LIBRARY_PATH=../python-can-3.3.4/can/interfaces/bmcan:$LD_LIBRARY_PATH
export PYTHONPATH=.:../python-can-3.3.4:$PYTHONPATH
python3 ruiwo_test.py
