import torch
import numpy as np
import nn as nn

# Função para carregar o modelo Actor
def load_actor(actor_path, input_dim, output_dim, device):
    actor = nn.PolicyNetwork(input_dim, output_dim).to(device)
    actor.load_state_dict(torch.load(actor_path,weights_only=True))
    actor.eval()  # Coloca o modelo em modo de avaliação
    return actor

def get_action_values(state, actor,device):
    
    with torch.no_grad():
        action, log_prob = actor.sample(state.to(device))
    return action.cpu().numpy()  

def get_action(input_data):

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    input_dim = 7  
    output_dim = 3
    # Caminho para o modelo salvo
    actor_path = "/home/luisc/ws_manipulator/src/manipulator/manipulator/rl/saved_models/sac_actor_EP101.pt"

    # Carregar o modelo Actor
    actor = load_actor(actor_path, input_dim, output_dim, device)

    input_tensor = torch.FloatTensor(input_data).to(device)

    action = get_action_values(input_tensor, actor, device)
    return action
