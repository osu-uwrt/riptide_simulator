using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class GameSystem : MonoBehaviour
{
  public static GameSystem Instance { get; private set; }
  public ROSConnection ros;
  public float unloadUnusedAssetsFrequency = 2.50f;

  void Awake()
  {
    if (Instance != null) {
      Debug.LogError("There is more than one instance!");
      return;
    }

    var host = Dns.GetHostEntry(Dns.GetHostName()); // Assign user's IP to ROSConnection prefab
      foreach (var ip in host.AddressList)
      {
        if (ip.AddressFamily == AddressFamily.InterNetwork)
        {
          ros.RosIPAddress = ip.ToString();
        }
      }
  }

  void Start() {
    Screen.fullScreen = true;
    StartCoroutine(unloadAssetsCoroutine());
  }

  IEnumerator unloadAssetsCoroutine() {
    while (true) {
      Resources.UnloadUnusedAssets();
      yield return new WaitForSeconds(unloadUnusedAssetsFrequency);
    }
  }
}

