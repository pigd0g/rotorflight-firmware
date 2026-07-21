FROM ubuntu:22.04

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install \
    build-essential \
    clang \
    libblocksruntime-dev \
    python3 \
    curl \
    git

RUN mkdir /rotorflight
WORKDIR /rotorflight