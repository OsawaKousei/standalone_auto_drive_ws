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

# Create minimal pyproject and install Python deps via uv add in /opt (not shadowed by bind mounts)
WORKDIR /opt/ad-viz
RUN printf '[project]\nname = "ad-viz-deps"\nversion = "0.0.0"\nrequires-python = ">=3.10"\n' > pyproject.toml && \
    uv venv --python /usr/bin/python3 && \
    uv add numpy matplotlib

ENV VIRTUAL_ENV=/opt/ad-viz/.venv
ENV PATH="/opt/ad-viz/.venv/bin:/root/.local/bin:${PATH}"
ENV Python3_ROOT_DIR=/opt/ad-viz/.venv

ENV CC=gcc-12 \
    CXX=g++-12 \
    CXXFLAGS="-std=c++20"

WORKDIR /workspace