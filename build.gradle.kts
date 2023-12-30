plugins {
    kotlin("jvm") version "1.9.0"
    kotlin("plugin.serialization") version "1.9.22"
    id("org.openjfx.javafxplugin") version "0.1.0"
    application
}

group = "org.example"
version = "1.0"

repositories {
    mavenCentral()
    maven("https://maven.brott.dev/")
}

dependencies {
    testImplementation(kotlin("test"))
    implementation("com.acmerobotics.roadrunner:core:0.5.5")
    implementation("org.jetbrains.kotlinx:kotlinx-serialization-json:1.6.0")
}

tasks.test {
    useJUnitPlatform()
}

kotlin {
    jvmToolchain(11)
}

javafx {
    version = "17.0.8"
    modules = listOf("javafx.controls")
}

application {
    mainClass.set("MainKt")
}