FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ビルドに必要な最小限のツールのみをインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gcc-12 \
    g++-12 \
    git \
    ca-certificates \
    curl \
    python3 \
    python3-dev \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Install uv and Python packages (system site-packages)
ENV PATH="/root/.local/bin:${PATH}"
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# Create project venv and install Python deps via uv (without pyproject)
WORKDIR /workspace
RUN uv venv --python /usr/bin/python3 && \
    uv pip install --python /workspace/.venv numpy matplotlib

ENV VIRTUAL_ENV=/workspace/.venv
ENV PATH="/workspace/.venv/bin:/root/.local/bin:${PATH}"
ENV Python3_ROOT_DIR=/workspace/.venv

ENV CC=gcc-12 \
    CXX=g++-12 \
    CXXFLAGS="-std=c++20"

WORKDIR /workspace