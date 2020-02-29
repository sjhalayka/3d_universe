
#include "main.h"

int main(int argc, char **argv)
{	
	get_vertices_and_triangles(100000);

	double K = 0;

	vector<float> edge_lengths;



	for (size_t i = 0; i < triangles.size(); i++)
	{
		if (tri_neighbours[i].size() != 3)
		{
			cout << "Error" << endl;
			return 1;
		}


		vector<float> e = get_tri_edge_lengths(triangles[i]);

		for (size_t j = 0; j < e.size(); j++)
			edge_lengths.push_back(e[j]);


		vector_3 a = vertices[triangles[i].vertex_indices[2]];
		vector_3 b = vertices[triangles[i].vertex_indices[1]];
		vector_3 c = vertices[triangles[i].vertex_indices[0]];

		vector_3 ab = a - b;
		vector_3 bc = b - c;
		
		vector_3 this_normal = (ab.cross(bc)).normalize();


		size_t neighbour_0_index = tri_neighbours[i][0];

		a = vertices[triangles[neighbour_0_index].vertex_indices[2]];
		b = vertices[triangles[neighbour_0_index].vertex_indices[1]];
		c = vertices[triangles[neighbour_0_index].vertex_indices[0]];

		ab = a - b;
		bc = b - c;

		vector_3 neighbour_0_normal = (ab.cross(bc)).normalize();


		size_t neighbour_1_index = tri_neighbours[i][1];

		a = vertices[triangles[neighbour_1_index].vertex_indices[2]];
		b = vertices[triangles[neighbour_1_index].vertex_indices[1]];
		c = vertices[triangles[neighbour_1_index].vertex_indices[0]];

		ab = a - b;
		bc = b - c;

		vector_3 neighbour_1_normal = (ab.cross(bc)).normalize();


		size_t neighbour_2_index = tri_neighbours[i][2];

		a = vertices[triangles[neighbour_2_index].vertex_indices[2]];
		b = vertices[triangles[neighbour_2_index].vertex_indices[1]];
		c = vertices[triangles[neighbour_2_index].vertex_indices[0]];

		ab = a - b;
		bc = b - c;
	
		vector_3 neighbour_2_normal = (ab.cross(bc)).normalize();


		double d_i = this_normal.dot(neighbour_0_normal) + this_normal.dot(neighbour_1_normal) + this_normal.dot(neighbour_2_normal);
		d_i /= 3.0;

		double k_i = (1.0 - d_i) / 2.0;

		K += k_i;
	}

	K /= static_cast<double>(triangles.size());

	cout << 2.0 + K << endl;


	write_histogram(edge_lengths, "histogram.png");



	return 0;





	glutInit(&argc, argv);
	init_opengl(win_x, win_y);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
	glutKeyboardFunc(keyboard_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutPassiveMotionFunc(passive_motion_func);
	//glutIgnoreKeyRepeat(1);

	glutMainLoop();

	glutDestroyWindow(win_id);

	return 0;
}
