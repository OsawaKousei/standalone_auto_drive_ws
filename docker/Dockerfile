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
    && rm -rf /var/lib/apt/lists/*

ENV CC=gcc-12 \
    CXX=g++-12 \
    CXXFLAGS="-std=c++20"

WORKDIR /workspace