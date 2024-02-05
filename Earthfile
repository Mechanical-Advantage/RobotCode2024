VERSION 0.7
FROM debian:bookworm-20240110
RUN apt-get update -y
WORKDIR /RobotCode2024

javabase:
    RUN apt-get install -y openjdk-17-jdk-headless git

ccbase:
    RUN apt-get install -y build-essential git cmake clang lld

ccbase-mingw-w64:
    FROM +ccbase
    RUN apt-get install -y mingw-w64

osxcross:
    FROM crazymax/osxcross:13.1-r0-debian
    SAVE ARTIFACT /osxcross

robotcode2024-dependencies:
    CACHE /root/.gradle
    FROM +javabase
    COPY gradle/ gradle/
    COPY settings.gradle settings.gradle
    COPY build.gradle build.gradle
    COPY .wpilib .wpilib
    COPY vendordeps vendordeps
    COPY gradlew gradlew
    RUN ./gradlew build --no-daemon
    SAVE IMAGE --cache-hint

robotcode2024-build:
    CACHE /root/.gradle
    FROM +robotcode2024-dependencies
    COPY src src
    RUN ./gradlew build --no-daemon
    SAVE IMAGE --cache-hint