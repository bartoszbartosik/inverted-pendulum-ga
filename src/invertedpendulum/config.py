import os
from dotenv import load_dotenv
from pathlib import Path


root = Path(__file__).parent.parent.parent
load_dotenv(dotenv_path=root / '.env.local')
load_dotenv(dotenv_path=root / '.env')

EXPERIMENTS_DIR = Path(os.getenv('EXPERIMENTS_DIR'))
CONFIGS_DIR = Path(os.getenv('CONFIGS_DIR'))
