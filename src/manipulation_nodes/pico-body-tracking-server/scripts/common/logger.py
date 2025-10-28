import logging
import os
from pathlib import Path
from logging.handlers import RotatingFileHandler
def setup_logger():
    logger = logging.getLogger('kuavo-pico-vr')
    logger.setLevel(logging.DEBUG)
    log_suffix = f'log/kuavo-pico-vr'
    log_dir = f'/var/{log_suffix}'
    try:
        # Check if we have write permission for /var directory
        if not os.access('/var/log/', os.W_OK):
            log_dir = f'./{log_suffix}'
        Path(log_dir).mkdir(parents=True, exist_ok=True)
    except Exception as e:
        # If creation in /var fails, create in current directory
        log_dir = f'./{log_suffix}'
        Path(log_dir).mkdir(parents=True, exist_ok=True)        
    log_file = f'{log_dir}/kuavo_pico_vr.log'
    # Create .gitignore if log directory is not in /var
    if not log_dir.startswith('/var'):
        gitignore_path = os.path.join(log_dir, '.gitignore')
        if not os.path.exists(gitignore_path):
            try:
                with open(gitignore_path, 'w') as f:
                    f.write('*\n')
            except Exception as e:
                print(f'Warning: Could not create .gitignore in {log_dir}: {e}')
    print(f'kuavo-pico-vr log_file: {log_file}')

    fh = RotatingFileHandler(log_file, maxBytes=2*1024*1024, backupCount=5)  # 每个日志文件最大 2 MB，保留 5 个备份文件
    fh.setLevel(logging.DEBUG)
    
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)
    
    logger.addHandler(fh)
    logger.addHandler(ch)
    return logger

def disable_sdk_logging():
    """
        Disable SDK logging.
    """
    logging.disable()


""" Logger """
SDKLogger = setup_logger()