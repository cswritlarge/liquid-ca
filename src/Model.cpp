#include "Model.hpp"

#include <numeric>   // std::accumulate
#include <algorithm> // std::for_each

Model::Model(double arena_size,
             int num_agents,
             double communication_range,
             int max_degree,
             int seed,
             double initial_density,
             double agent_speed) :
   _communication_range(communication_range),
   _max_degree(max_degree),
   _rng(seed),
   _stats(num_agents),
   _noise(0.0),
   _arena_size(arena_size),
   go_interactive_(1.0),
   go_dark_(0.0)
{
   std::uniform_real_distribution<double> coordinate_distribution(-arena_size/2, arena_size/2);
   std::uniform_real_distribution<double> heading_distribution(0, 2*M_PI);
   std::bernoulli_distribution state_distribution(initial_density);
   std::uniform_int_distribution<int> seed_distribution;

   for(int i = 0; i < num_agents; i++)
   {
      Point initial_position(coordinate_distribution(_rng), coordinate_distribution(_rng));
      Heading initial_heading(heading_distribution(_rng));
      Agent a(initial_position, initial_heading, agent_speed, arena_size, seed_distribution(_rng));
      _agents.push_back(a);
      if(state_distribution(_rng))
      {
         _agent_states.push_back(1);
     }
      else
      {
         _agent_states.push_back(0);
      }
   }
   _turn_distribution = heading_distribution;
   _step_distribution = std::uniform_int_distribution<int>(1,1);
   _stats.PushState(CurrentDensity(), CurrentNetwork());
}

Model::~Model() {}

void Model::SetPositionalState(double initial_density)
{
   double x_threshold = (_arena_size / 2.0) - (_arena_size * (1.0 - initial_density));
   _stats = ModelStats(_agents.size());
   for(int i = 0; i < _agents.size(); i++)
   {
      if(_agents[i].Position().GetX() <= x_threshold)
      {
         _agent_states[i] = 1;
      }
      else
      {
         _agent_states[i] = 0;
      }
   }
   _stats.PushState(CurrentDensity(), CurrentNetwork());
}

void Model::RecordNetworkDensityOnly()
{
   _stats.NetworkSummaryOnly();
}

size_t Model::int2size_t(int val)
{
   return (val <= INT_MAx) ? (int)((ssize_t)val) : -1
}

double Model::CurrentDensity() const
{
   return std::accumulate(_agent_states.begin(), _agent_states.end(), 0.0) / _agent_states.size();
}

std::shared_ptr<NetworkSnapshot> Model::CurrentNetwork() const
{
   std::shared_ptr<NetworkSnapshot> snapshot = std::make_shared<NetworkSnapshot>(_agents.size());	     for(int i = 0; i < _agents.size(); i++)
   {	
      // temp vector of potential edges for i
      typedef std::vector<std::pair<int, int>> Neighbors;
      Neighbors neighbors;
      for(int j = i+1; j < _agents.size(); j++)
      {
         if(_agents[i].Position().Within(_communication_range, _agents[j].Position()))
         {
            /*if(_conn_type == 'o')
            {
               snapshot->AddEdge(i, j);
            }
            else
            {*/		 
	    neighbors.push_back(std::make_pair(i,j)); //j,i rather than i,j hopefully allows me to use sort() on the value of j
	    /*for (Neighbors::size_type l=0; l<neighbors.size(); l++)
	    {   std::cout << neighbors.at(l).first << std::endl;}
            //snapshot->AddEdge(i, j);   // AddEdge already adds i->j and j->i edge if I remember correctly...
            }*/
         }
      }
   // loop through neighbor list with
      for(int k =  0; k < neighbors.size(); k++)
      {
         /*Node degree limited connections
         if(_conn_type == 'n')
         {*/
         //size_t num_nodes = _max_degree;
         //int num_nodes = _max_degree;
	 size_t num_nodes = int2size_t(_max_degree);
	 // randomly shuffle neighbors
         std::random_device rd;
         std::mt19937 g(rd());
         std::shuffle(neighbors.begin(), neighbors.end(), g);
         //int it = 0;
	 size_t it = 0;
         while(it < num_nodes)
         {
            //int u = std::get<0>(neighbors[it]);
            //int v = std::get<1>(neighbors[it]);
            auto u = neighbors.at(it).first;
            auto v = neighbors.at(it).second;
            snapshot->AddEdge(u, v);
            // neighbors.erase(index of chosen tuple);
            it++;
         }
      }
         /* Rentian connections
         else if(_conn_type == 'r')
         {
            connect build edge i, j with probability inversely propotional to distance between i and j
         }
       }*/
   }
   return snapshot;
}

const ModelStats& Model::GetStats() const
{
   return _stats;
}

const std::vector<Agent>& Model::GetAgents() const
{
   return _agents;
}

const std::vector<int>& Model::GetStates() const
{
   return _agent_states;
}

void Model::SetMovementRule(std::shared_ptr<MovementRule> rule)
{
   for(auto& agent : _agents)
   {
      agent.SetMovementRule(rule->Clone());
   }
}

void Model::SetNoise(double p)
{
   _noise_probability = p;
   _noise = std::bernoulli_distribution(fabs(p));
}

void Model::SetPDark(double p)
{
   go_dark_ = std::bernoulli_distribution(fabs(p));

   for(auto& agent : _agents)
   {
      if(go_dark_(_rng))
      {
         agent.GoDark();
      }
   }
}

void Model::SetPInteractive(double p)
{
   go_interactive_ = std::bernoulli_distribution(fabs(p));
}

int Model::Noise(int i)
{
   if(_noise(_rng))
   {
      return 1 - i;
   }
   else
   {
      return i;
   }
}

void Model::Step(const Rule* rule)
{
   for(Agent& agent : _agents)
   {
      agent.Step();
      if(agent.IsInteractive() && go_dark_(_rng))
      {
         agent.GoDark();
      }
      else if(agent.IsDark() && go_interactive_(_rng))
      {
         agent.GoInteractive();
      }
   }

   std::shared_ptr<NetworkSnapshot> current_network = CurrentNetwork();
   std::vector<int> new_states(_agents.size());
   for(int a = 0; a < _agent_states.size(); a++)
   {
      if(_agents[a].IsInteractive())
      {
         auto neighbors = current_network->GetNeighbors(a);
         std::vector<int> neighbor_states;
         for(int n : neighbors)
         {
            if(_agents[n].IsInteractive())
            {
               if(_noise_probability < 0.0) {
                  if(!_noise(_rng))
                  {
                     neighbor_states.push_back(_agent_states[n]);
                  }
               }
               else
               {
                  neighbor_states.push_back(Noise(_agent_states[n]));
               }
            }
         }
         std::pair<int, double> update = rule->Apply(_agent_states[a], neighbor_states);
         new_states[a] = update.first;
         _agents[a].SetHeading(_agents[a].GetHeading() + Heading(update.second));
      }
      else
      {
         new_states[a] = _agent_states[a];
      }
   }
   _agent_states = new_states;
   _stats.PushState(CurrentDensity(), current_network);
}
