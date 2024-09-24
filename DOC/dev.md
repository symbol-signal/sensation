# DEV notes
## Sync project to Raspberry
`rsync -avz --delete -e ssh --exclude '.git' --exclude '.idea' --exclude venv . pi@rpibathroom-14:/home/pi/sensation`

## Install on raspberry in editable mode
```commandline
python -m venv sensation_venv
source sensation_venv/bin/activate
cd sensation
flit install --symlink
```
