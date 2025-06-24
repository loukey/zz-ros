# 1. Check current memory & swap usage
free -h

# 2. Create a swap file (e.g. 4 GiB)
sudo fallocate -l 12G /swapfile
# If your filesystem doesn’t support fallocate, use:
# sudo dd if=/dev/zero of=/swapfile bs=1M count=4096

# 3. Lock it down
sudo chmod 600 /swapfile

# 4. Format it as swap
sudo mkswap /swapfile

# 5. Enable it now
sudo swapon /swapfile
