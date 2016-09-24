package com.afton.hello;

import java.rmi.RemoteException;

public class Main {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Hello h = new Hello();
		try {
			h.forward(1);
		} catch (RemoteException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
