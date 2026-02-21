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
    && rm -rf /var/lib/apt/lists/*

# 3. uv のインストール (公式推奨のマルチステージビルド形式でクリーンに取得)
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# ビルドには引き続き GCC 12 を使用
ENV CC=gcc-12 \
    CXX=g++-12 \
    CXXFLAGS="-std=c++20"

WORKDIR /workspace