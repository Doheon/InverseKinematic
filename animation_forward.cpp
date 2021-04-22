#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <shader.h>
#include <sphere1.h>
#include <spline.h>
#include <linkjoint.h>
#include <viewbvh.h>


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mode);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_pos_callback(GLFWwindow* window, double xpos, double ypos);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);


void drawnode(linkjoint root, Shader spline, int elindices);

unsigned int loadTexture(const char* path);


using namespace std;
float swt = 1;
float fov = 45.0f;

bool droneview = false;

glm::vec3 cameraPos = glm::vec3(0.0f, 100.0f, 220.0f);
glm::vec3 dronefront = glm::vec3(1.0f, 0.0f, -1.0f);

glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
glm::vec3 centerofrot(0.0f, 0.0f, 0.0f);

glm::vec3 dronepos = glm::vec3(-50, 0, 50);

glm::vec3 posvec;
glm::vec3 rotate_axis;
float rotate_theta;



int main() {
	//-------------------------------여기부터-------------------------------------
	glfwInit(); //glfw초기화
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);//(우리가 설정할 옵션, 옵션값)
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(800, 600, "LearnOpenGL", NULL, NULL); //(높이, 너비,창이름, null, null) 객체생성

	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window); //window를 주context로 지정
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback); //창의 크기가 변경될때마다 함수를 호출하도록 등록

	//GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))//glad를 초기화
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}
	//--------------------------여기까지 no touch (기본설정)--------------------------

	float square[] = {
-1.0f, 0.0f, -1.0f, 0.0f,  1.0f, 0.0f,
 1.0f, 0.0f, -1.0f, 0.0f,  1.0f, 0.0f,
 1.0f, 0.0f,  1.0f, 0.0f,  1.0f, 0.0f,
 1.0f, 0.0f,  1.0f, 0.0f,  1.0f, 0.0f,
-1.0f, 0.0f,  1.0f, 0.0f,  1.0f, 0.0f,
-1.0f, 0.0f, -1.0f, 0.0f,  1.0f, 0.0f,
	};

	//square--------------------------------------
	unsigned int squarevao, squarevbo;
	glGenBuffers(1, &squarevbo); //버퍼 id를 생성
	glGenVertexArrays(1, &squarevao);

	glBindVertexArray(squarevao); //array 바인딩
	glBindBuffer(GL_ARRAY_BUFFER, squarevbo);//버퍼의 유형을 선택하여 바인딩
	glBufferData(GL_ARRAY_BUFFER, sizeof(square), square, GL_STATIC_DRAW);//데이터를 버퍼의 메모리에 저장

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);	//vertex 좌표
	//texture attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);	//normal vector 좌표


	//sphere------------------------------------------------------
	value vec;
	vec = sphere(1, 40);
	vector<float> ver = vec.vertices;
	vector<int> ind = vec.indices;
	vector<int> ind_grid = vec.indices_grid;

	unsigned int sphereVbo;
	unsigned int sphereEbo;
	unsigned int sphereVao;
	//색칠-----------------------
	glGenBuffers(1, &sphereVbo);
	glGenBuffers(1, &sphereEbo);
	glGenVertexArrays(1, &sphereVao);

	glBindVertexArray(sphereVao);
	glBindBuffer(GL_ARRAY_BUFFER, sphereVbo);
	glBufferData(GL_ARRAY_BUFFER, ver.size() * sizeof(float), &ver[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphereEbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ind.size() * sizeof(float), &ind[0], GL_STATIC_DRAW);


	//propeller-----------------------------------------------------------------------------------------------------------------------
	int ct = 4; //단면을 구성하는 control point 개수
	int step1 = 30; //단면쪼개는횟수
	int step2 = 5; //swept쪼개는 횟수

	int clockwise = 1;//단면의 좌표들이 배치된 방향 (시계방향이면 -1)


	sweptsurface element("element.txt", ct, step2, step1, clockwise);
	vector<float> elcoord = element.sweptcoord;
	vector<float> elnormal = element.normalcoord;
	vector<int> elindices = element.paintindices;
	unsigned int elvbo, elvbo2, elvao, elebo;

	//color
	glGenBuffers(1, &elvbo);
	glGenBuffers(1, &elvbo2);
	glGenBuffers(1, &elebo);
	glGenVertexArrays(1, &elvao);

	glBindVertexArray(elvao);
	glBindBuffer(GL_ARRAY_BUFFER, elvbo);
	glBufferData(GL_ARRAY_BUFFER, elcoord.size() * sizeof(float), &elcoord[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, elvbo2);
	glBufferData(GL_ARRAY_BUFFER, elnormal.size() * sizeof(float), &elnormal[0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, elindices.size() * sizeof(float), &elindices[0], GL_STATIC_DRAW);


	//-------------------------------------------------------------------------------------------------------------------------



	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	glfwSetCursorPosCallback(window, mouse_pos_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetKeyCallback(window, key_callback);


	Shader spline("shader/spline.vs", "shader/spline.fs");

	spline.use();

	//material information
	glm::vec3 diffusecolor;
	glm::vec3 specularcolor;
	float alpha;
	float shininess;

	//light information
	glm::vec3 lightcolor(1.0f);

	glm::vec3 diffuselight;
	glm::vec3 ambientlight;
	glm::vec3 specularlight;

	//direct light
	glm::vec3 directlightcolor = glm::vec3(0.0f);
	glm::vec3 direction(-1.0f, -1.0f, 0.0f);
	diffuselight = directlightcolor * glm::vec3(0.8f);
	ambientlight = directlightcolor * glm::vec3(0.2f);
	specularlight = directlightcolor * glm::vec3(0.7f);

	glUniform3fv(glGetUniformLocation(spline.ID, "direct.ambient"), 1, &ambientlight[0]);
	glUniform3fv(glGetUniformLocation(spline.ID, "direct.diffuse"), 1, &diffuselight[0]);
	glUniform3fv(glGetUniformLocation(spline.ID, "direct.specular"), 1, &specularlight[0]);
	glUniform3fv(glGetUniformLocation(spline.ID, "direct.direction"), 1, &direction[0]);


	//point light

	glm::vec3 pointlightcolor[] = {
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 0.6f, 0.6f)
	};

	glm::vec3 diffuselightcolor[] = {
	 pointlightcolor[0] * glm::vec3(0.8f),
	 pointlightcolor[1] * glm::vec3(0.8f),
	 pointlightcolor[2] * glm::vec3(0.8f)
	};

	glm::vec3 ambientlightcolor[] = {
	 pointlightcolor[0] * glm::vec3(0.3f),
	 pointlightcolor[1] * glm::vec3(0.3f),
	 pointlightcolor[2] * glm::vec3(0.0f)
	};

	glm::vec3 specularlightcolor[] = {
	 pointlightcolor[0] * glm::vec3(0.7f),
	 pointlightcolor[1] * glm::vec3(0.7f),
	 pointlightcolor[2] * glm::vec3(0.7f)
	};

	float constant = 1.0f;
	float linear = 0.09f;
	float quadratic = 0.032f;

	glm::vec3 pointlightPos[] = {
		glm::vec3(-90.0f,  100.0f,  -50.0f),
		glm::vec3(90.0f, 70.0f, 50.0f),
		glm::vec3(20.0f, 30.0f, -80.0f)
	};

	for (int i = 0; i < 3; i++) {
		string p;
		p = "point[" + to_string(i) + "]";
		glUniform3fv(glGetUniformLocation(spline.ID, (p + ".ambient").c_str()), 1, &ambientlightcolor[i][0]);
		glUniform3fv(glGetUniformLocation(spline.ID, (p + ".diffuse").c_str()), 1, &diffuselightcolor[i][0]);
		glUniform3fv(glGetUniformLocation(spline.ID, (p + ".specular").c_str()), 1, &specularlightcolor[i][0]);
		glUniform3fv(glGetUniformLocation(spline.ID, (p + ".position").c_str()), 1, &pointlightPos[i][0]);
		glUniform1f(glGetUniformLocation(spline.ID, (p + ".constant").c_str()), constant);
		glUniform1f(glGetUniformLocation(spline.ID, (p + ".linear").c_str()), linear);
		glUniform1f(glGetUniformLocation(spline.ID, (p + ".quadratic").c_str()), quadratic);
	}

	//-------------------------------------------------------------------------


	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//render loop
	while (!glfwWindowShouldClose(window))//루프가 시작될 때마다 종료하도록 지시되었는지 확인, 그렇다면 true반환
	{
		float currenttime = glfwGetTime();
		processInput(window); //esc가 눌렸는지 루프마다 확인

		glClearColor(0.6f, 0.85f, 0.9f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 view = glm::mat4(1.0f);
		//view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		view = glm::lookAt(cameraPos, centerofrot, cameraUp);

		glm::mat4 projection = glm::mat4(1.0f);
		projection = glm::perspective(glm::radians(fov), (float)800 / 600, 0.1f, 500.0f);

		glm::mat4 model = glm::mat4(1.0f);

		//---------------------------------------------------------------------------
		spline.use();
		glUniformMatrix4fv(glGetUniformLocation(spline.ID, "view"), 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(glGetUniformLocation(spline.ID, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
		glUniform3fv(glGetUniformLocation(spline.ID, "viewPos"), 1, &cameraPos[0]);
		glUniform1f(glGetUniformLocation(spline.ID, "shadow"), 1.0f);

		//---------------------------------------------------------------------------


		//sphere--------------------------------------------------------------------
		/*
		//material information
		glm::vec3 diffusecolor = glm::vec3(10.0f, 10.0f, 10.0f);
		glm::vec3 specularcolor = glm::vec3(0.0f, 0.0f, 0.0f);
		alpha = 1;
		shininess = 64;
		glUniform1f(glGetUniformLocation(spline.ID, "material.shininess"), shininess);
		glUniform1f(glGetUniformLocation(spline.ID, "material.alpha"), alpha);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.diffuse"), 1, &diffusecolor[0]);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.specular"), 1, &specularcolor[0]);

		for (glm::vec3 a : pointlightPos) {
			model = glm::mat4(1.0f);
			model = glm::translate(model, a);
			model = glm::scale(model, glm::vec3(2.0f, 2.0f, 2.0f));
			glUniformMatrix4fv(glGetUniformLocation(spline.ID, "model"), 1, GL_FALSE, glm::value_ptr(model));
			glBindVertexArray(sphereVao);
			glDrawElements(GL_TRIANGLES, ind.size(), GL_UNSIGNED_INT, NULL);
		}
		*/
		//-------------------------------------------------------------------------

		//element--------------------------------------------------------------------
		glBindVertexArray(elvao);

		//material information
		diffusecolor = glm::vec3(2.0f, 2.0f, 2.0f);
		specularcolor = glm::vec3(1.0f, 1.0f, 1.0f);
		float alpha = 1;
		float shininess = 64;
		glUniform1f(glGetUniformLocation(spline.ID, "material.shininess"), shininess);
		glUniform1f(glGetUniformLocation(spline.ID, "material.alpha"), alpha);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.diffuse"), 1, &diffusecolor[0]);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.specular"), 1, &specularcolor[0]);
		glUniform3fv(glGetUniformLocation(spline.ID, "viewPos"), 1, &cameraPos[0]);

		currenttime *= 1.5;

		//matrix
		mat4 i = glm::mat4(1.0f);
		mat4 tr = glm::translate(i, vec3(0,50 + 10*sin(4*currenttime),0));
		mat4 sc = glm::scale(i, vec3(2.5f, 1.5f,2.5f));

		//body
		linkjoint body(0, 1, 22 , tr, sc);

		//head
		tr = glm::translate(i, vec3(0, 43, 0));
		sc = glm::scale(i, vec3(2.0f, 0.5f, 2.0f));
		linkjoint head(0, 1, 22, tr, sc);
		body.insertnode(&head);


		//waist
		sc = glm::scale(i, vec3(2.5f, 0.5f, 2.5f));
		linkjoint waist(0, 1, 22, i, sc);
		body.insertnode(&waist);

		
		//arm
		float theta1 = sin(currenttime*2);
		float theta2 = sin(currenttime*2) - 1;
		float theta3 = sin(3.1415f + currenttime*2);
		float theta4 = sin(currenttime*2 + 3.1415f) - 1;

		tr = glm::translate(i, vec3(7, 28, 0));
		sc = glm::scale(i, vec3(1.0f, 0.7f, 1.0f));

		linkjoint arm1_1(theta1, 1, 22, tr);
		linkjoint arm1_2(theta2, 1, 22,i, sc);
		body.insertnode(&arm1_1);
		arm1_1.insertnode(&arm1_2);

		tr = glm::translate(i, vec3(-7, 28, 0));
		linkjoint arm2_1(theta3, 1, 22, tr);
		linkjoint arm2_2(theta4, 1, 22, i, sc);
		body.insertnode(&arm2_1);
		arm2_1.insertnode(&arm2_2);

		//hand
		sc = glm::scale(i, vec3(0.8f, 0.2f, 0.8f));
		linkjoint hand1(0, 1, 22, i, sc);
		linkjoint hand2(0, 1, 22, i, sc);
		arm1_2.insertnode(&hand1);
		arm2_2.insertnode(&hand2);


		//leg
		tr = glm::translate(i, vec3(3, 0, 0));
		sc = glm::scale(i, vec3(1.0f, 1.3f, 1.0f));
		linkjoint leg1_1(theta3, 1, 22, tr,sc);
		linkjoint leg1_2((sin(currenttime*3) +1)/2, 1);


		waist.insertnode(&leg1_1);
		leg1_1.insertnode(&leg1_2);


		tr = glm::translate(i, vec3(-3, 0, 0));
		sc = glm::scale(i, vec3(1.0f, 1.3f, 1.0f));
		linkjoint leg2_1(theta1, 1, 22, tr, sc);
		linkjoint leg2_2((sin(currenttime*3 + 4.7124f) + 1)/2, 1);
		waist.insertnode(&leg2_1);
		leg2_1.insertnode(&leg2_2);

		//foot
		tr = glm::translate(i, vec3(0, 0, 3));
		sc = glm::scale(i, vec3(0.7f, 0.3f, 1.8f));
		linkjoint foot1(0, 1, 22, tr, sc);
		linkjoint foot2(0, 1, 22, tr, sc);
		leg1_2.insertnode(&foot1);
		leg2_2.insertnode(&foot2);
		


		drawnode(body, spline, elindices.size());



		//plane---------------------------------------------------------------------------
		/*
		diffusecolor = glm::vec3(1.3f, 1.1f, 0.8f);
		specularcolor = glm::vec3(0.0f, 0.0f, 0.0f);
		alpha = 1;
		shininess = 64;
		glUniform1f(glGetUniformLocation(spline.ID, "material.shininess"), shininess);
		glUniform1f(glGetUniformLocation(spline.ID, "material.alpha"), alpha);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.diffuse"), 1, &diffusecolor[0]);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.specular"), 1, &specularcolor[0]);

		glBindVertexArray(squarevao);
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.0f, -10.1f, 0.0f));
		model = glm::scale(model, glm::vec3(150.0f));
		glUniformMatrix4fv(glGetUniformLocation(spline.ID, "model"), 1, GL_FALSE, glm::value_ptr(model));
		//glDrawArrays(GL_TRIANGLES, 0, 36);
		*/
		//---------------------------------------------------------------------------------

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}





void framebuffer_size_callback(GLFWwindow* window, int width, int height) //창의 크기를 조정하는 함수 (객체, 너비, 높이)
{
	glViewport(0, 0, width, height); //윈도우의 왼쪽 아래 모서리의 위치, 너비와 높이의 픽셀 (-1~1 사이의 좌표값을 원하는 값의 픽셀스케일로 변환)
}


void mouseclick_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
		swt = -swt + 1;
}

int temp = 1;
glm::vec3 firstvec;
glm::vec3 secondvec;
glm::vec3 xvec;
glm::quat rotate_quat(1, 0, 0, 0);
glm::vec3 firstcamera;

glm::vec3 posvec2;
float lastX, lastY;
bool firstmouse = true;

float yaw = -90.0f;
float pitch = 0.0f;

void mouse_pos_callback(GLFWwindow* window, double xpos, double ypos)
{

	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		posvec.x = (xpos - 400);
		posvec.y = (300 - ypos);
		posvec.z = 0;

		if (temp == 1) {
			xvec = glm::normalize(glm::cross(vec3(0,1,0), cameraPos - centerofrot));
			firstvec = cameraPos - centerofrot + xvec * posvec.x + glm::normalize(glm::cross(cameraPos - centerofrot, xvec)) * posvec.y;
			temp = 0;
		}

		secondvec = cameraPos - centerofrot + xvec * posvec.x + glm::normalize(glm::cross(cameraPos - centerofrot, xvec)) * posvec.y;

		if (firstvec != secondvec) {
			glm::vec3 cross = glm::cross(secondvec, firstvec);
			rotate_axis = glm::normalize(cross);
			rotate_theta = atan2(glm::length(cross), glm::dot(firstvec, secondvec));
			rotate_quat = glm::angleAxis(rotate_theta, rotate_axis);
			cameraPos = rotate_quat * (cameraPos - centerofrot) * glm::inverse(rotate_quat) + centerofrot;
			//cameraUp = glm::normalize(glm::cross(cameraPos - centerofrot, xvec));
			temp = 1;
		}
	}

	else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
	{
		temp = 1;
	}

	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
		if (firstmouse)
		{
			lastX = xpos;
			lastY = ypos;
			firstmouse = false;
			firstvec = cameraPos - centerofrot;
		}
		float xoffset = -(xpos - lastX) * glm::length(cameraPos) * 0.0013;
		float yoffset = -(lastY - ypos) * glm::length(cameraPos) * 0.0013;
		lastX = xpos;
		lastY = ypos;

		posvec2 = xoffset * glm::normalize(glm::cross(vec3(0, 1, 0), firstvec)) + yoffset * glm::normalize(glm::cross(firstvec, glm::normalize(glm::cross(vec3(0, 1, 0), firstvec))));
		cameraPos += posvec2;
		centerofrot += posvec2;
		//cameraUp = glm::normalize(glm::cross(firstvec, glm::normalize(glm::cross(vec3(0, 1, 0), firstvec))));
	}

	else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE)
	{
		firstmouse = true;
	}

}


void processInput(GLFWwindow* window)
{
	glm::vec3 axis(0, 1, 0);
	float theta = 0.02f;
	glm::quat quat = glm::angleAxis(theta, axis);

	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == 1)
		glfwSetWindowShouldClose(window, true);

	float cameraSpeed = 1.0f * 0.02f;

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) //dolly in
		cameraPos -= cameraSpeed * (cameraPos - centerofrot);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) //dolly out
		cameraPos += cameraSpeed * (cameraPos - centerofrot);

}


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_C && action == GLFW_PRESS) {
			cameraPos = glm::vec3(0.0f, 100.0f, 220.0f);
			centerofrot = glm::vec3(0.0f);
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mode)
{


}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	if (fov >= 1.0f && fov <= 45.0f)
		fov -= yoffset;
	if (fov <= 1.0f)
		fov = 1.0f;
	if (fov >= 45.0f)
		fov = 45.0f;
}

void drawnode(linkjoint root, Shader spline, int elindices) {

	mat4 m;
	m = root.model;
	glUniformMatrix4fv(glGetUniformLocation(spline.ID, "model"), 1, GL_FALSE, glm::value_ptr(m));
	glDrawElements(GL_TRIANGLES, elindices, GL_UNSIGNED_INT, NULL);

	for (linkjoint* a:root.node) {
		m = (*a).model;
		glUniformMatrix4fv(glGetUniformLocation(spline.ID, "model"), 1, GL_FALSE, glm::value_ptr(m));
		glDrawElements(GL_TRIANGLES, elindices, GL_UNSIGNED_INT, NULL);
		drawnode(*a, spline, elindices);
	}
}