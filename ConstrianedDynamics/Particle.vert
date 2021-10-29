#version 330 core
layout (location = 0) in vec3 aPos;


uniform mat4 projection;
uniform mat4 model;
out vec4 pos;

void main()
{   
    // note that we read the multiplication from right to left
    gl_Position = projection * model * vec4(aPos, 1.0);
}