#ifndef _SAMPLE_DATA_H
#define _SAMPLE_DATA_H

struct SampleData {
	SampleData() :
		q_data(NULL), qdot_data(NULL), qddot_data(NULL), tau_data(NULL)
	{}
	~SampleData() {
		delete_data();
	}

	RigidBodyDynamics::Math::VectorNd *q_data;
	RigidBodyDynamics::Math::VectorNd *qdot_data;
	RigidBodyDynamics::Math::VectorNd *qddot_data;
	RigidBodyDynamics::Math::VectorNd *tau_data;

	void delete_data() {
		if (q_data)
			delete[] q_data;
		q_data = NULL;

		if (qdot_data)
			delete[] qdot_data;
		qdot_data = NULL;

		if (qddot_data)
			delete[] qddot_data;
		qddot_data = NULL;

		if (tau_data)
			delete[] tau_data;
		tau_data = NULL;
	}

	void fill_random_data (int q_size, int qdot_size, int sample_count) {
		delete_data();

		q_data = new RigidBodyDynamics::Math::VectorNd[sample_count];
		qdot_data = new RigidBodyDynamics::Math::VectorNd[sample_count];
		qddot_data = new RigidBodyDynamics::Math::VectorNd[sample_count];
		tau_data = new RigidBodyDynamics::Math::VectorNd[sample_count];

		for (int si = 0; si < sample_count; si++) {
			q_data[si].resize (q_size);
			qdot_data[si].resize (qdot_size);
			qddot_data[si].resize (qdot_size);
			tau_data[si].resize (qdot_size);

			for (int i = 0; i < q_size; i++) 
				q_data[si][i] = (rand() / RAND_MAX) * 2. -1.;

			for (int i = 0; i < qdot_size; i++) {
				qdot_data[si][i] = (rand() / RAND_MAX) * 2. -1.;
				qddot_data[si][i] = (rand() / RAND_MAX) * 2. -1.;
				tau_data[si][i] = (rand() / RAND_MAX) * 2. -1.;
			}
		}
	}
};

#endif
