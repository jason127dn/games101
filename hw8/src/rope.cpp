#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        //        Comment-in this part when you implement the constructor
        Vector2D dp = (end-start)/(num_nodes-1);
        for (int i=0;i<num_nodes;i++)
        {
            Mass* ms = new Mass(start+i*dp,node_mass,false);
            masses.push_back(ms);
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
        for(int i=1;i<num_nodes;i++)
        {
            Spring* sp = new Spring(masses[i-1],masses[i],k);
            springs.push_back(sp);
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node

            Vector2D diff = (s->m1->position-s->m2->position);
        
            std::cout << "for:" << s->m1->forces << " :force\n";
            std::cout << "for:" << s->m2->forces << " :force\n";
            //std::cout<< "diff" << diff <<" :diff\n";
            Vector2D f = -s->k * diff/diff.norm()*(diff.norm()-s->rest_length);
            //std::cout<< "foc" << f <<" :foc\n";
            s->m1->forces+=f;
            s->m2->forces-=f;
            std::cout << "for:" << s->m1->forces << " :force\n";
            std::cout << "for:" << s->m2->forces << " :force\n";
            std::cout << "==sp==\n";
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces+=gravity*m->mass;

                // TODO (Part 2): Add global damping
                m->velocity= m->velocity*(1.-0.00005)+m->forces/m->mass*delta_t;
                m->position+=m->velocity*delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        
            Vector2D diff = (s->m1->position-s->m2->position);
            Vector2D f = -s->k * diff/diff.norm()*(diff.norm()-s->rest_length);
            s->m1->forces+=f;
            s->m2->forces-=f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
