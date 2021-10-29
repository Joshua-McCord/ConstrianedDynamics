#version 330 core
out vec4 FragColor;
  
in float dist;  // Maybe Use Later
in vec4 finalPos;

void main()
{
    //202, 223, 158
    FragColor = vec4(202/250.0f, 223/250.0f, 158/250.0f, 1.0f);
} 