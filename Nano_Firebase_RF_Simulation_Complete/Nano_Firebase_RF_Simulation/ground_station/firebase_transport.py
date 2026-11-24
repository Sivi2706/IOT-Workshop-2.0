"""Authenticated Firebase Realtime Database REST transport for bench-test telemetry.

Firebase Email/Password users are restricted by Realtime Database Security Rules.
No service-account admin credentials, unauthenticated writes, or onboard Wi-Fi.
"""
from __future__ import annotations

import os
import time
from pathlib import Path
from urllib.parse import urlsplit

import requests

try:
    from dotenv import load_dotenv
except ImportError:
    load_dotenv = None

# Local .env is optional, but never ship an actual .env file to others.
if load_dotenv:
    load_dotenv(Path(__file__).resolve().parents[1] / '.env')


def required(key: str) -> str:
    value = os.getenv(key, '').strip()
    if not value:
        raise ValueError(f'Missing {key}. Set it in the project .env file or environment.')
    return value


def station_path(station: str) -> str:
    if not station or not station.replace('-', '').replace('_', '').isalnum():
        raise ValueError('Station must contain only letters, digits, hyphens and underscores')
    return f'workshop/rocket/{station}'


class FirebaseREST:
    """REST client with refreshable Firebase Authentication ID token."""
    def __init__(self, role: str, timeout: float = 6.0):
        if role not in ('PUBLISHER', 'VIEWER'):
            raise ValueError('role must be PUBLISHER or VIEWER')
        self.database_url = required('FIREBASE_DATABASE_URL').rstrip('/')
        parsed = urlsplit(self.database_url)
        if parsed.scheme != 'https' or not parsed.netloc or parsed.path not in ('', '/'):
            raise ValueError('FIREBASE_DATABASE_URL must be the HTTPS root URL of Realtime Database')
        self.api_key = required('FIREBASE_API_KEY')
        self.email = required(f'FIREBASE_{role}_EMAIL')
        self.password = required(f'FIREBASE_{role}_PASSWORD')
        self.session = requests.Session()
        self.timeout = timeout
        self.id_token = None
        self.refresh_token = None
        self.expiry = 0.0
        self.uid = None

    @staticmethod
    def _json_response(response, context):
        # Never surface a requests.HTTPError URL containing the Firebase ID token.
        if not response.ok:
            try:
                details = response.json().get('error', {})
                if isinstance(details, dict):
                    details = details.get('message', 'Access denied')
            except ValueError:
                details = 'Request rejected'
            raise RuntimeError(f'{context}: HTTP {response.status_code} — {str(details)[:160]}')
        if response.status_code == 204 or not response.content:
            return None
        try:
            return response.json()
        except ValueError as exc:
            raise RuntimeError(f'{context}: invalid JSON response') from exc

    def _post(self, endpoint, payload, context):
        try:
            resp = self.session.post(endpoint, json=payload, timeout=self.timeout)
        except requests.RequestException as exc:
            raise ConnectionError(f'{context}: network error ({type(exc).__name__})') from None
        return self._json_response(resp, context)

    def _authenticate(self):
        url = ('https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword'
               f'?key={self.api_key}')
        data = self._post(url,
                          {'email': self.email, 'password': self.password,
                           'returnSecureToken': True}, 'Firebase sign-in')
        self.id_token = data['idToken']
        self.refresh_token = data['refreshToken']
        self.uid = data.get('localId')
        self.expiry = time.monotonic() + max(60, int(data['expiresIn'])) - 90

    def _refresh(self):
        url = f'https://securetoken.googleapis.com/v1/token?key={self.api_key}'
        try:
            resp = self.session.post(url, data={
                'grant_type': 'refresh_token', 'refresh_token': self.refresh_token},
                timeout=self.timeout)
        except requests.RequestException as exc:
            raise ConnectionError(f'Firebase token refresh: {type(exc).__name__}') from None
        data = self._json_response(resp, 'Firebase token refresh')
        self.id_token = data['id_token']
        self.refresh_token = data['refresh_token']
        self.expiry = time.monotonic() + max(60, int(data['expires_in'])) - 90

    def _token(self):
        if self.id_token is None:
            self._authenticate()
        elif time.monotonic() >= self.expiry:
            try:
                self._refresh()
            except (RuntimeError, ConnectionError):
                self._authenticate()
        return self.id_token

    def request(self, method: str, path: str, *, params=None, payload=None):
        """Use auth=token in RTDB URL per Firebase REST API; never log token URLs."""
        if '..' in path or path.startswith('/'):
            raise ValueError('Invalid Firebase path')
        query = dict(params or {})
        query['auth'] = self._token()
        try:
            response = self.session.request(
                method, f'{self.database_url}/{path}.json',
                params=query, json=payload, timeout=self.timeout)
        except requests.RequestException as exc:
            raise ConnectionError(f'Firebase database: network error ({type(exc).__name__})') from None
        # Expired or revoked tokens may need a fresh login.
        if response.status_code == 401:
            self.id_token = None
            query['auth'] = self._token()
            try:
                response = self.session.request(
                    method, f'{self.database_url}/{path}.json',
                    params=query, json=payload, timeout=self.timeout)
            except requests.RequestException as exc:
                raise ConnectionError(f'Firebase database retry: {type(exc).__name__}') from None
        return self._json_response(response, 'Firebase Realtime Database')

    def close(self):
        self.session.close()
