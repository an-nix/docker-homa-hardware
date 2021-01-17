FROM alpine:latest

ENV PYTHONUNBUFFERED=1
RUN apk add --update --no-cache python3 && ln -sf python3 /usr/bin/python
RUN python3 -m ensurepip
RUN pip3 install --no-cache --upgrade pip setuptools
RUN pip3 install paho-mqtt
RUN pip3 install install RPi.bme280

COPY main.py /main.py

CMD ["/main.py"]
ENTRYPOINT [ "python3" ]
