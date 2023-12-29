plugins {
    kotlin("jvm") version "1.9.0"
    id("org.openjfx.javafxplugin") version "0.0.9"
    application
}

group = "org.example"
version = "1.0-SNAPSHOT"

repositories {
    mavenCentral()
    maven("https://maven.brott.dev/")
}

dependencies {
    testImplementation(kotlin("test"))
    implementation("com.acmerobotics.roadrunner:core:0.5.5")
}

tasks.test {
    useJUnitPlatform()
}

kotlin {
    jvmToolchain(8)
}

javafx {
    version = "17.0.8"
    modules = listOf("javafx.controls")
}

application {
    mainClass.set("MainKt")
}