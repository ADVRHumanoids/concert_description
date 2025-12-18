# install concert_isaac if needed
pip show concert_isaac || pip install -e /workspace/concert_description/concert_isaac/source

# run the simulation script
cd /workspace/concert_description/concert_isaac/scripts/simulation
python simulate.py