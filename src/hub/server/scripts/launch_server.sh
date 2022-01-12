#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR
cd ..

source env/bin/activate

# comment this out when it's time to go live
export FLASK_ENV=development
export FLASK_APP=server
python3 -m flask run --host=35.0.45.26 --port=8080
