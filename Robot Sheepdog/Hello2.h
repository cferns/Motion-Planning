/*SheepDog - Motion Planning Final Project
CODE'd on 6 December 2015

details about the code:
bodies[0] represents the dog
bodies[i] => 1<i<numSheep represents the i'th sheep, numSheep is max number of sheep
'All control variables' have been purposely defined in the begining , which help to control the position, velocity and forces of the dog and sheep

When a sheep is within sphere of influence of dog, force is applied onto sheep, the distance/speed by which a sheep moves depends upon the magnitude of force applied on the sheep (can be controlled by: force_magnitude_factor) and the time for which the force is applied(time_steps)
Force is applied for certain time (controlled by: timesteps) in a biased manner; in which for time =0.9timesteps sheep moves opposite the dog and for the remaining time =0.1 x timesteps, sheep moves towards goal

At certain instances, it was observed that the sheep sticks to the wall due to forces acting towards the wall. To prevent this, arrangment has been made so that when sheep reaches within 1 unit distance from the wall, sheep is forced to move away towards any point (currently, 15,35)

//Although the time_steps is in seconds, the actual time taken(to display motion of objects) is scaled with the frequency of display which is 60Hz by default and can be changed from the engine graphic display window
*/

#ifndef FOOTEST_H
#define FOOTEST_H
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define pi 3.14285714286
#include<math.h>
#include<iostream>

//All control variables
b2Vec2 goal(22.5,37.5);
float dogX = 0,dogY=20;
float sheepX =-15,sheepY=20;
const int numSheep=10;//check code
int remaining_time_steps[numSheep];//same as numSheep
int tempON[numSheep];
int temp[numSheep];
int countemp[numSheep];
float sheepVelocityFactor=1.5;//multiplication factor for sheep velocity
float dog_velocity_factor=5;//multiplication factor for dog velocity
float force_magnitude_factor=12;//multiplication factor for force applied onto sheep at dog influence sphere
int time_steps=160;//time steps for which force will be applied after sheep is at sphere of influence
int dog_sphere_radius = 5;//apply force on sheep if within sphere
int back_point=3;//move dog to back of sheep by htis distance
int for_loop_counter;


class FooTest : public Test
    {
    b2Body* bodies[1+numSheep];    //+6 for sheep; +1 for dog
    float theta,theta1,difference;
    int a,count;
    float abs_dist;
    float abs_disttemp[];
    int far_sheep_num;

    b2Vec2 pos; //used for position of sheep body

    public:
    FooTest()
        {
        m_world->SetGravity(b2Vec2(0,0));
            //body definitions
            {
            //Sheep body
            b2BodyDef sheep_bodyDef;
            sheep_bodyDef.type = b2_dynamicBody;
            //shape
            b2PolygonShape sheep_polygonShape;
            b2Vec2 verticCes[5];
            verticCes[0].Set(-0.35,  0.35);
            verticCes[1].Set(-0.35,  -0.35);
            verticCes[2].Set( 0.5, -0.35);
            verticCes[3].Set( 0.75,  0);
            verticCes[4].Set( 0.5,  0.35);
            sheep_polygonShape.Set(verticCes, 5);
            //fixture
            b2FixtureDef sheep_FixtureDef;
            sheep_FixtureDef.shape = &sheep_polygonShape;
            sheep_FixtureDef.density = 1;
            //Sheep bodies
            for (int i=1;i<=numSheep;i++)
                {
                float x_ = -17 + 34 * (rand()/(float)RAND_MAX);
                float y_ = 37 * (rand()/(float)RAND_MAX);
                //sheep_bodyDef.position.Set(sheepX+i*2, sheepY+i*2);
                sheep_bodyDef.position.Set(x_,y_);
                bodies[i] = m_world->CreateBody(&sheep_bodyDef);
                bodies[i]->CreateFixture(&sheep_FixtureDef);
                }


            //Dog body
            b2BodyDef dog_bodyDef;
            dog_bodyDef.type = b2_dynamicBody;
            //shape
            b2PolygonShape dog_polygonShape;
            b2Vec2 vertices[7];
            vertices[0].Set(-0.75,  0.25);
            vertices[1].Set(-0.75,  -0.25);
            vertices[2].Set(0,  -0.6);
            vertices[3].Set( 0.5, -0.5);
            vertices[4].Set( 0.75,  0);
            vertices[5].Set( 0.5,  0.5);
            vertices[6].Set(0,  0.6);
            dog_polygonShape.Set(vertices, 7);
            //fixture
            b2FixtureDef dog_FixtureDef;
            dog_FixtureDef.shape = &dog_polygonShape;
            dog_FixtureDef.density = 1;
            //add body to world
            float x_ = -18 + 38 *(rand() % 100 + 1)/100;
            float y_ = 38 *(rand() % 100 + 1)/100;
            dog_bodyDef.position.Set(x_,y_);
            bodies[0] = m_world->CreateBody(&dog_bodyDef);
            bodies[0]->CreateFixture(&dog_FixtureDef);
            //add sensor//just to show influence region
/*
            b2FixtureDef dog_sensor_fixtureDef;
            b2CircleShape circleShape;
            circleShape.m_radius = dog_sphere_radius;
            dog_sensor_fixtureDef.shape = &circleShape;
            dog_sensor_fixtureDef.isSensor = true;
            bodies[0]->CreateFixture(&dog_sensor_fixtureDef);
*/

            //Boundary
            b2BodyDef boundary_bodyDef;
            boundary_bodyDef.type= b2_staticBody;
            //b2Body* walls = m_world->CreateBody(&boundary_bodyDef);
            b2PolygonShape boundary_polygonShape;
            b2FixtureDef boundary_fixtureDef;
            boundary_fixtureDef.shape = &boundary_polygonShape;
            boundary_fixtureDef.density=1;
            //bottom boundary
            boundary_bodyDef.angle=0;
            boundary_bodyDef.position.Set(0, 0);
            boundary_polygonShape.SetAsEdge( b2Vec2(-20,0), b2Vec2(20,0) );
            m_world->CreateBody(&boundary_bodyDef)->CreateFixture(&boundary_fixtureDef);
            //right boundary
            boundary_bodyDef.position.Set(20, 0);
            boundary_polygonShape.SetAsEdge( b2Vec2(0,0), b2Vec2(0,35) );
            m_world->CreateBody(&boundary_bodyDef)->CreateFixture(&boundary_fixtureDef);
            //left boundary
            boundary_bodyDef.position.Set(-20, 0);
            boundary_polygonShape.SetAsEdge( b2Vec2(0,0), b2Vec2(0,40) );
            m_world->CreateBody(&boundary_bodyDef)->CreateFixture(&boundary_fixtureDef);
            //top boundary
            boundary_bodyDef.position.Set(0, 40);
            boundary_polygonShape.SetAsEdge( b2Vec2(-20,0), b2Vec2(20,0) );
            m_world->CreateBody(&boundary_bodyDef)->CreateFixture(&boundary_fixtureDef);
            //top pan
            boundary_bodyDef.position.Set(20, 40);
            boundary_polygonShape.SetAsEdge( b2Vec2(0,0), b2Vec2(5,0) );
            m_world->CreateBody(&boundary_bodyDef)->CreateFixture(&boundary_fixtureDef);
            //down pan
            boundary_bodyDef.position.Set(20, 35);
            boundary_polygonShape.SetAsEdge( b2Vec2(0,0), b2Vec2(5,0) );
            m_world->CreateBody(&boundary_bodyDef)->CreateFixture(&boundary_fixtureDef);
            //right pan
            boundary_bodyDef.position.Set(25, 35);
            boundary_polygonShape.SetAsEdge( b2Vec2(0,0), b2Vec2(0,5) );
            m_world->CreateBody(&boundary_bodyDef)->CreateFixture(&boundary_fixtureDef);


            //Obstacle bodies
            b2BodyDef obstacle_BodyDef;
            obstacle_BodyDef.type = b2_staticBody;
            //shape
            b2CircleShape obstacle_Shape;
            obstacle_Shape.m_p.Set(0, 0);
            //fixtures
            b2FixtureDef obstacle_FixtureDef;
            obstacle_FixtureDef.isSensor = false;
            //obstacle_FixtureDef.shape = &obstacle_Shape;
            obstacle_FixtureDef.density = 1;
            for(int i=0 ;i<6; i++)
                {
                obstacle_Shape.m_radius = 0.5+0.5*(rand() % 100 + 1)/100;
                obstacle_FixtureDef.shape = &obstacle_Shape;
                float x_ = -16 + 32 *(rand() % 100 + 1)/100;
                float y_ = 4 + 32 *(rand() % 100 + 1)/100;
                obstacle_BodyDef.position.Set(x_, y_);
                obstacle_BodyDef.angle = 0;
                b2Body* obstacle= m_world->CreateBody(&obstacle_BodyDef);
                obstacle->CreateFixture(&obstacle_FixtureDef);
                }
            }
            //end of body definitions
        }

    void generate_random_direction_velocity_for_sheep(void)
        {
        for (int i=1;i<=(numSheep);i++)
            {
            theta = bodies[i]->GetAngle();
            if(theta>=0)
                {
                while(theta>6.28319)
                theta=theta-6.28319;
                }
            else
                {
                while(theta<0)
                theta=6.28319+theta;
                }
            pos = bodies[i]->GetPosition();
            bodies[i]->SetLinearVelocity(b2Vec2(sheepVelocityFactor*cos(theta),sheepVelocityFactor*sin(theta)));

            if(theta1!=0)  			//Sheep reached fence, turning in the opposite direction.
                {
                difference=theta1-theta;
                if(difference<0)
                    difference *= -1;
                if(difference<0.1)
                    theta1=0;
                }
            else if((pos.x<-18)||(pos.x>18))	// if Sheep is close to vertical fence
                {
                if(((sin(theta)>0)&&(cos(theta)>0))&&(pos.x>18))    //First quadrant angle
                    {
                    theta1=3.14159-theta;
                    bodies[i]->SetAngularVelocity(5);
                    }
                else if((sin(theta)>0)&&(cos(theta)<0)&&(pos.x<-18))  //Second quadrant angle
                    {
                    theta1=theta-1.57079;
                    theta1=1.57079-theta1;
                    bodies[i]->SetAngularVelocity(-5);
                    }
                else if((sin(theta)<0)&&(cos(theta)<0)&&(pos.x<-18))	//Third quadrant angle
                    {
                    theta1=theta-3.14159;
                    theta1=6.28319-theta1;
                    bodies[i]->SetAngularVelocity(5);
                    }
                else if((sin(theta)<0)&&(cos(theta)>0)&&(pos.x>18))	//Forth quadrant angle
                    {
                    theta1=6.28319-theta;
                    theta1=3.14159+theta1;
                    bodies[i]->SetAngularVelocity(-5);
                    }
                }
            else if((pos.y<2)||(pos.y>38))	//if sheep is close to horizontal fence
                {
                if(((sin(theta)>0)&&(cos(theta)>0))&&(pos.y>38))    //First quadrant angle
                        {
                        theta1=6.28319-theta;
                        bodies[i]->SetAngularVelocity(-5);
                        }
                else if((sin(theta)>0)&&(cos(theta)<0)&&(pos.y>38))  //Second quadrant angle
                        {
                        theta1=3.14159-theta;
                        theta1=3.14159+theta1;
                        bodies[i]->SetAngularVelocity(5);
                        }
                else if((sin(theta)<0)&&(cos(theta)<0)&&(pos.y<2))    //Third quadrant angle
                        {
                        theta1=theta-3.14159;
                        theta1=3.14159-theta1;
                        bodies[i]->SetAngularVelocity(-5);
                        }
                else if((sin(theta)<0)&&(cos(theta)>0)&&(pos.y<2))     //Forth quadrant angle
                        {
                        theta1=6.28319-theta;
                        bodies[i]->SetAngularVelocity(5);
                        }
                }
            else
                {
                bodies[i]->SetAngularVelocity(((rand()%80)-40)/10);
                }
            }
            //end of for loop
        }

    void set_direction_velocity_for_dog(int far_sheep)
        {
        b2Vec2 bodyPos = bodies[far_sheep]->GetPosition();
        float x_new, y_new, x_new2, y_new2, dist, dist2,lineSlope,lineConst;
        lineSlope=(goal.y-bodyPos.y)/(goal.x-bodyPos.x);
        lineConst=goal.y-(lineSlope*goal.x);
        y_new=bodyPos.y-back_point;
        x_new=(y_new-lineConst)/lineSlope;
        dist=((bodyPos.y-y_new)*(bodyPos.y-y_new)+(bodyPos.x-x_new)*(bodyPos.x-x_new));
        x_new2=bodyPos.x-back_point;
        y_new2=lineSlope*x_new2+lineConst;
        dist2=((bodyPos.y-y_new2)*(bodyPos.y-y_new2)+(bodyPos.x-x_new2)*(bodyPos.x-x_new2));
        b2Vec2 newDogPosn;
        if (dist<dist2)
            newDogPosn = b2Vec2(x_new,y_new);
        else
            newDogPosn = b2Vec2(x_new2,y_new2);
        b2Vec2 toTarget = newDogPosn - bodies[0]->GetPosition();
        float desiredAngle = atan2f( -toTarget.x, toTarget.y );
        toTarget.Normalize();

        bodies[0]->SetTransform( bodies[0]->GetPosition(), desiredAngle+90*DEGTORAD );
        bodies[0]->SetLinearVelocity(dog_velocity_factor*toTarget);
        }

    void apply_impulse_to_sheep_towards_point(int sheep_num, float px, float py, float bx, float by)
        {
        b2Vec2 goalVect(px,py);
        b2Vec2 toTarget = goalVect - bodies[sheep_num]->GetPosition();
        toTarget *= force_magnitude_factor*0.3;
        bodies[sheep_num]->ApplyForce( b2Vec2(toTarget), bodies[sheep_num]->GetWorldPoint(b2Vec2(bx,by)) );
        generate_random_direction_velocity_for_sheep();
        }

    void apply_impulse_to_sheep_opposite_dog(int sheep_num)
        {
        b2Vec2 toTarget = bodies[sheep_num]->GetPosition() - bodies[0]->GetPosition();
        toTarget *= force_magnitude_factor;
        bodies[sheep_num]->ApplyForce( b2Vec2(toTarget), bodies[sheep_num]->GetWorldPoint(b2Vec2(0.75,0)) );
        generate_random_direction_velocity_for_sheep();
        }

    int call_farthest_sheep()
        {
        float max_dist=0;
        for (int k=1;k<=numSheep;k++)
            {
            if (temp[k]==1)
                {continue;}
            b2Vec2 posA = bodies[k]->GetPosition();
            //b2Vec2 distance_btw = bodies[j]->GetPosition() - bodies[0]->GetPosition();
            abs_disttemp[k] = (posA.x-goal.x)*(posA.x-goal.x)+(posA.y-goal.y)*(posA.y-goal.y);
            if (abs_disttemp[k]>max_dist)
                {max_dist=abs_disttemp[k];
                far_sheep_num=k;}
            }
        return far_sheep_num;
        }

    void apply_force_on_sheep()
        {
        for (int j=1;j<=numSheep;j++)
            {
            b2Vec2 posA = bodies[j]->GetPosition();
            b2Vec2 posB = bodies[0]->GetPosition();
            //b2Vec2 distance_btw = bodies[j]->GetPosition() - bodies[0]->GetPosition();
            abs_dist = (posA.x-posB.x)*(posA.x-posB.x)+(posA.y-posB.y)*(posA.y-posB.y);
            float tempDist=dog_sphere_radius*dog_sphere_radius;

            if (abs_dist < tempDist and tempON[j]==0)   //4*4
                {
                tempON[j]=1;
                remaining_time_steps[j]=time_steps;
                }

            if (remaining_time_steps[j] > time_steps*0.1)
                {
                apply_impulse_to_sheep_opposite_dog(j);
                remaining_time_steps[j]--;

                if (remaining_time_steps[j]==1)
                    {
                    tempON[j]=0;
                    }
                }
            else if (remaining_time_steps[j]>0)
                {
                apply_impulse_to_sheep_towards_point(j,(goal.x-1),(goal.y-1),0.75,0);
                remaining_time_steps[j]--;

                if (remaining_time_steps[j]==1)
                    {
                    tempON[j]=0;
                    }
                }
            }
        }

    void prevent_sheep_sticking_to_wall()
        {
        for (int j=1;j<=numSheep;j++)
            {
            b2Vec2 posA = bodies[j]->GetPosition();
            if (posA.x>20)
                {
                temp[j] =1;
                }

            if ((posA.x<-19 or( posA.x>19 and posA.x<20 ) or posA.y<1 or (posA.y>39 and posA.y<40) ) and countemp[j]==0)
                {
                countemp[j]=time_steps*0.4;
                }
            if (countemp[j]>0)
                {
                apply_impulse_to_sheep_towards_point(j,15,35,0.2,0);
                countemp[j]--;
                }
            }
        }

    void Step(Settings* settings)
        {
        Test::Step(settings);

        generate_random_direction_velocity_for_sheep();

        far_sheep_num=call_farthest_sheep();

        set_direction_velocity_for_dog(far_sheep_num);

        apply_force_on_sheep();

        prevent_sheep_sticking_to_wall();
        }

    static Test* Create()
        {
            return new FooTest;
        }
    };

#endif
