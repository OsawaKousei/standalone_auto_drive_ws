FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# 基本ツールのインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    gcc-12 \
    g++-12 \
    git \
    ca-certificates \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common \
    libvulkan1 \
    mesa-vulkan-drivers \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# Same toolchain as the development container.
ARG RUST_VERSION=1.99.1
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo
ENV PATH="${CARGO_HOME}/bin:${PATH}"
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/rustup-init.sh && \
    sh /tmp/rustup-init.sh -y --profile minimal --default-toolchain "${RUST_VERSION}" --no-modify-path && \
    rm /tmp/rustup-init.sh

# 3. uv のインストール (公式推奨のマルチステージビルド形式でクリーンに取得)
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# ビルドには引き続き GCC 12 を使用
ENV CC=gcc-12 \
    CXX=g++-12 \
    CXXFLAGS="-std=c++20"

WORKDIR /workspace
