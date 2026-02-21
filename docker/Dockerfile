FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# 基本ツールのインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gcc-12 \
    g++-12 \
    git \
    ca-certificates \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common \
    python3 \
    python3-dev \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# LLVM公式リポジトリから Clang 17 をインストール
# (Ubuntu 22.04標準のClang 14はGCC12のrangesヘッダーと相性が悪いため)
RUN wget https://apt.llvm.org/llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh 17 && \
    rm llvm.sh && \
    apt-get install -y clang-tidy-17 clang-format-17

# update-alternatives でシステム標準のコマンドとして登録
RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-17 100 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-17 100 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-17 100 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-17 100

# Install uv and Python packages
ENV PATH="/root/.local/bin:${PATH}"
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# Python setup
WORKDIR /opt/ad-viz
RUN uv init && \
    uv venv --python /usr/bin/python3 && \
    uv add numpy matplotlib

ENV VIRTUAL_ENV=/opt/ad-viz/.venv
ENV PATH="/opt/ad-viz/.venv/bin:/root/.local/bin:${PATH}"
ENV Python3_ROOT_DIR=/opt/ad-viz/.venv

# ビルドには引き続き GCC 12 を使用
ENV CC=gcc-12 \
    CXX=g++-12 \
    CXXFLAGS="-std=c++20"

WORKDIR /workspace