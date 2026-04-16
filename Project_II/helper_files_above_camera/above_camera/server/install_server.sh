#!/bin/bash
set -e

# Update and install ffmpeg
echo "Installing ffmpeg..."
sudo apt update
sudo apt install -y ffmpeg

# Download and install mediamtx
echo "Installing mediamtx..."
# Detect architecture (assuming arm64 or armv7 based on Pi model, defaulting to arm64 for newer Pis)
echo "Installing mediamtx..."

ARCH=$(dpkg --print-architecture)

if [ "$ARCH" = "amd64" ]; then
    MEDIAMTX_ARCH="amd64"
elif [ "$ARCH" = "armhf" ]; then
    MEDIAMTX_ARCH="armv7"
elif [ "$ARCH" = "arm64" ]; then
    MEDIAMTX_ARCH="arm64"
else
    echo "Unknown architecture: $ARCH"
    exit 1
fi

MEDIAMTX_VERSION="v1.16.1"

wget "https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/mediamtx_${MEDIAMTX_VERSION}_linux_${MEDIAMTX_ARCH}.tar.gz"

tar -xzf "mediamtx_${MEDIAMTX_VERSION}_linux_${MEDIAMTX_ARCH}.tar.gz"
sudo mv mediamtx /usr/local/bin/
sudo cp mediamtx.yml /usr/local/etc/mediamtx.yml
rm mediamtx_${MEDIAMTX_VERSION}_linux_${MEDIAMTX_ARCH}.tar.gz

# Create mediamtx service
echo "Creating mediamtx service..."
cat <<EOF | sudo tee /etc/systemd/system/mediamtx.service
[Unit]
Description=MediaMTX RTSP Server
After=network.target

[Service]
ExecStart=/usr/local/bin/mediamtx /usr/local/etc/mediamtx.yml
Restart=always

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable mediamtx
sudo systemctl start mediamtx

echo "Installation complete!"
